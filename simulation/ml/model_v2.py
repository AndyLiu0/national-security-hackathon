"""ARGUS v2 — architecture mirrors `architecture.svg`.

The four physical EO viewports (F/R/B/L) are stitched into a SINGLE 360°
panoramic strip before the CNN. Same for the four IR viewports. Each
panorama is shaped (T, 4_quadrants, 6_channels) and fed to a 2D CNN whose
azimuth axis uses CIRCULAR PADDING — so the convolution wraps around the
F→R→B→L→F seam exactly as a real 360° view would. The CNN therefore sees
each band as one continuous image instead of four disconnected feeds.

Pipeline:

    [360° EO panorama   (T, 4, 6)]  -> 2D-CNN (circular pad on azimuth) ┐
    [360° IR panorama   (T, 4, 6)]  -> 2D-CNN (circular pad on azimuth) ┼-> 3 modality tokens
    [Air-pressure       (T, 6)   ]  -> MLP                              ┘
                                                  -> Cross-modal Transformer
                                                  -> MLP Head
                                                  -> Pose (Δx,Δy,Δz) + Velocity (vx,vy,vz)

Per-frame feature vector layout (54 dims, see sim_physics.FEATURE_NAMES):
    pressure : indices 0..5
    SWIR pano: indices 6..29   (4 quadrants × 6 channels)
    EO   pano: indices 30..53  (4 quadrants × 6 channels)
"""
from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F

N_CAMS = 4
PANO_CHANNELS = 6      # per-quadrant feature channels
PRESSURE_DIM = 6

INPUT_DIM = PRESSURE_DIM + N_CAMS * PANO_CHANNELS * 2  # = 54

PRESSURE_SLICE = (0, PRESSURE_DIM)
SWIR_SLICE = (PRESSURE_DIM, PRESSURE_DIM + N_CAMS * PANO_CHANNELS)
EO_SLICE   = (SWIR_SLICE[1], SWIR_SLICE[1] + N_CAMS * PANO_CHANNELS)

SEQ_LEN = 16
HORIZON = 10
TRAJ_OUT_DIM = 6
TRAJ_DT = 1.0

D_MODEL = 96
N_HEADS = 4
N_LAYERS = 2
FFN_DIM = 192


class _PanoramaCNN(nn.Module):
    """2D CNN over (time, azimuth) with circular padding on the azimuth axis.

    Input  : (B, T, N_CAMS, C)        — a panoramic strip per timestep.
    Output : (B, d_model)              — a single token summarising the band.

    Circular padding makes the conv kernel see quadrant L next to quadrant F,
    so a target straddling the F/L seam is handled correctly.
    """

    def __init__(self, in_ch: int, d_model: int, az_size: int = N_CAMS):
        super().__init__()
        self.az_size = az_size
        self.conv1 = nn.Conv2d(in_ch, 32, kernel_size=(3, 3))
        self.conv2 = nn.Conv2d(32, 64, kernel_size=(3, 3))
        self.conv3 = nn.Conv2d(64, d_model, kernel_size=(3, 3))
        self.act = nn.GELU()

    def _circ_pad(self, h: torch.Tensor) -> torch.Tensor:
        # (B, C, T, A): replicate-pad time, circular-pad azimuth.
        h = F.pad(h, (1, 1, 0, 0), mode="circular")
        h = F.pad(h, (0, 0, 1, 1), mode="replicate")
        return h

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B, T, A, C) -> (B, C, T, A)
        h = x.permute(0, 3, 1, 2).contiguous()
        h = self.act(self.conv1(self._circ_pad(h)))
        h = self.act(self.conv2(self._circ_pad(h)))
        h = self.act(self.conv3(self._circ_pad(h)))
        # Global pool over (time, azimuth) → (B, d_model)
        h = h.mean(dim=(2, 3))
        return h


class _PressureMLP(nn.Module):
    def __init__(self, in_ch: int, d_model: int):
        super().__init__()
        self.frame = nn.Sequential(
            nn.Linear(in_ch, 64),
            nn.GELU(),
            nn.Linear(64, d_model),
            nn.GELU(),
        )

    def forward(self, x):
        # x: (B, T, F) -> mean over T -> (B, d_model)
        return self.frame(x).mean(dim=1)


class ArgusFusionNet(nn.Module):
    def __init__(self,
                 seq_len: int = SEQ_LEN,
                 horizon: int = HORIZON,
                 d_model: int = D_MODEL,
                 n_heads: int = N_HEADS,
                 n_layers: int = N_LAYERS,
                 ffn_dim: int = FFN_DIM):
        super().__init__()
        self.seq_len = seq_len
        self.horizon = horizon

        self.cam_cnn = _PanoramaCNN(PANO_CHANNELS, d_model)
        self.ir_cnn = _PanoramaCNN(PANO_CHANNELS, d_model)
        self.pressure_mlp = _PressureMLP(PRESSURE_DIM, d_model)

        self.modality_emb = nn.Parameter(torch.randn(3, d_model) * 0.02)

        layer = nn.TransformerEncoderLayer(
            d_model=d_model,
            nhead=n_heads,
            dim_feedforward=ffn_dim,
            activation="gelu",
            batch_first=True,
            norm_first=True,
            dropout=0.05,
        )
        self.transformer = nn.TransformerEncoder(layer, num_layers=n_layers)
        self.norm = nn.LayerNorm(d_model)

        self.head = nn.Sequential(
            nn.Linear(d_model, ffn_dim),
            nn.GELU(),
            nn.Linear(ffn_dim, horizon * TRAJ_OUT_DIM),
        )
        self.head_cls = nn.Linear(d_model, 1)
        self.head_rng_spd = nn.Linear(d_model, 2)

    def _split(self, x):
        B, T, _ = x.shape
        pr = x[..., PRESSURE_SLICE[0]:PRESSURE_SLICE[1]]                # (B, T, 6)
        ir = x[..., SWIR_SLICE[0]:SWIR_SLICE[1]].reshape(B, T, N_CAMS, PANO_CHANNELS)
        cam = x[..., EO_SLICE[0]:EO_SLICE[1]].reshape(B, T, N_CAMS, PANO_CHANNELS)
        return cam, ir, pr

    def forward(self, x):
        cam, ir, pr = self._split(x)
        cam_tok = self.cam_cnn(cam)
        ir_tok = self.ir_cnn(ir)
        pr_tok = self.pressure_mlp(pr)
        tokens = torch.stack([cam_tok, ir_tok, pr_tok], dim=1) + self.modality_emb.unsqueeze(0)
        h = self.transformer(tokens)
        h = self.norm(h.mean(dim=1))
        traj = self.head(h).reshape(-1, self.horizon, TRAJ_OUT_DIM)
        return {
            "cls": self.head_cls(h).squeeze(-1),
            "rng_spd": self.head_rng_spd(h),
            "traj": traj,
        }
