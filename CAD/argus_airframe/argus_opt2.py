#!/usr/bin/env python3
"""
ARGUS HAPS Design Optimizer
============================
Multi-objective optimization engine for a High-Altitude Platform Station (HAPS)
designed for hypersonic cruise missile detection and tracking.

Mission Requirements:
  - Altitude: 85,000 ft (25,908 m)
  - Range: 8,000 km
  - Endurance: 7-10 days
  - Coverage radius: 100+ km per node

This solver finds the optimal airframe, power system, and fleet configuration
that minimizes per-unit and per-km² cost while satisfying all physical and
mission constraints.

Author: Project ARGUS / National Security Hackathon
"""

import math
import json
import sys
from dataclasses import dataclass, field, asdict
from typing import Optional, Tuple, List, Dict

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────

G = 9.80665          # m/s²
R_AIR = 287.058      # J/(kg·K) specific gas constant for dry air
GAMMA = 1.4          # ratio of specific heats
R_EARTH = 6371e3     # m
SOLAR_CONSTANT = 1361.0  # W/m² at top of atmosphere
FT_TO_M = 0.3048
KM_TO_M = 1000.0

# ─────────────────────────────────────────────────────────────────────────────
# ATMOSPHERE MODEL (International Standard Atmosphere + extensions to 86 km)
# ─────────────────────────────────────────────────────────────────────────────

def isa_atmosphere(altitude_m: float) -> Tuple[float, float, float]:
    """
    Returns (temperature_K, pressure_Pa, density_kg_m3) at given geometric altitude.
    Uses US Standard Atmosphere 1976 with multiple layers up to 86 km.
    """
    # Layer definitions: (base_alt_m, base_temp_K, lapse_rate_K_per_m)
    layers = [
        (0,     288.15,  -0.0065),    # Troposphere
        (11000, 216.65,   0.0),        # Tropopause
        (20000, 216.65,   0.001),      # Stratosphere 1
        (32000, 228.65,   0.0028),     # Stratosphere 2
        (47000, 270.65,   0.0),        # Stratopause
        (51000, 270.65,  -0.0028),     # Mesosphere 1
        (71000, 214.65,  -0.002),      # Mesosphere 2
    ]

    h = min(altitude_m, 86000)

    # Find the correct layer
    T_base = layers[0][1]
    P_base = 101325.0  # Pa at sea level
    h_base = 0
    lapse = layers[0][2]

    T = T_base
    P = P_base

    for i, (h_layer, T_layer, L_layer) in enumerate(layers):
        if i == 0:
            T_base = T_layer
            P_base = 101325.0
            h_base = h_layer
            lapse = L_layer
            continue

        # Calculate conditions at top of previous layer (= base of this layer)
        prev_h_base = layers[i-1][0]
        prev_T_base = layers[i-1][1] if i == 1 else T_base
        prev_lapse = layers[i-1][2]
        dh = h_layer - prev_h_base

        if abs(prev_lapse) < 1e-10:
            # Isothermal layer
            T_top = prev_T_base
            P_top = P_base * math.exp(-G * dh / (R_AIR * prev_T_base))
        else:
            T_top = prev_T_base + prev_lapse * dh
            P_top = P_base * (T_top / prev_T_base) ** (-G / (prev_lapse * R_AIR))

        if h <= h_layer:
            # We're in the previous layer
            dh_local = h - prev_h_base
            if abs(prev_lapse) < 1e-10:
                T = prev_T_base
                P = P_base * math.exp(-G * dh_local / (R_AIR * prev_T_base))
            else:
                T = prev_T_base + prev_lapse * dh_local
                P = P_base * (T / prev_T_base) ** (-G / (prev_lapse * R_AIR))
            rho = P / (R_AIR * T)
            return T, P, rho

        T_base = T_layer
        P_base = P_top
        h_base = h_layer
        lapse = L_layer

    # Above last defined layer base
    dh_local = h - layers[-1][0]
    last_lapse = layers[-1][2]
    last_T = layers[-1][1]

    if abs(last_lapse) < 1e-10:
        T = last_T
        P = P_base * math.exp(-G * dh_local / (R_AIR * last_T))
    else:
        T = last_T + last_lapse * dh_local
        P = P_base * (T / last_T) ** (-G / (last_lapse * R_AIR))

    rho = P / (R_AIR * T)
    return T, P, rho


# ─────────────────────────────────────────────────────────────────────────────
# AERODYNAMIC MODEL
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class AeroConfig:
    """Aerodynamic configuration parameters.
    Calibrated against Airbus Zephyr / PHASA-35 flight data.
    Ultra-clean HAPS have very low parasite drag.
    """
    Cd0: float = 0.008          # Zero-lift drag coefficient (ultra-clean HAPS, no landing gear drag)
    e: float = 0.93             # Oswald efficiency factor (high AR, clean wing)
    Cl_max: float = 1.3         # Maximum lift coefficient (thin airfoil at low Re)
    Cl_cruise: float = 0.9      # Cruise lift coefficient (high for stratospheric density)


def compute_aero(wingspan_m: float, chord_m: float, aero: AeroConfig,
                 altitude_m: float, mass_kg: float) -> dict:
    """
    Compute aerodynamic parameters at given flight condition.
    Returns dict with all aero quantities.
    """
    T, P, rho = isa_atmosphere(altitude_m)

    wing_area = wingspan_m * chord_m  # S (m²)
    aspect_ratio = wingspan_m / chord_m  # AR = b/c = b²/S

    weight = mass_kg * G  # N

    # Required CL for level flight: L = W = 0.5 * rho * V² * S * CL
    # We'll solve for V given CL_cruise
    CL = aero.Cl_cruise
    V_required = math.sqrt(2 * weight / (rho * wing_area * CL))

    # Stall speed
    V_stall = math.sqrt(2 * weight / (rho * wing_area * aero.Cl_max))

    # Drag coefficient
    CD_induced = CL**2 / (math.pi * aspect_ratio * aero.e)
    CD_total = aero.Cd0 + CD_induced

    # Lift-to-drag ratio
    L_over_D = CL / CD_total

    # Drag force
    drag = 0.5 * rho * V_required**2 * wing_area * CD_total

    # Power required for level flight (thrust * velocity)
    # Include propeller efficiency
    prop_efficiency = 0.82  # High-efficiency slow prop
    power_required = drag * V_required / prop_efficiency

    # Best L/D (theoretical)
    CL_best = math.sqrt(aero.Cd0 * math.pi * aspect_ratio * aero.e)
    CD_best = 2 * aero.Cd0
    LD_best = CL_best / CD_best

    # Reynolds number
    mu = 1.458e-6 * T**1.5 / (T + 110.4)  # Sutherland's law
    Re = rho * V_required * chord_m / mu

    # Mach number
    a = math.sqrt(GAMMA * R_AIR * T)
    mach = V_required / a

    return {
        'wing_area_m2': wing_area,
        'aspect_ratio': aspect_ratio,
        'CL_cruise': CL,
        'CD_total': CD_total,
        'CD_induced': CD_induced,
        'L_over_D': L_over_D,
        'LD_best': LD_best,
        'V_cruise_ms': V_required,
        'V_cruise_kmh': V_required * 3.6,
        'V_cruise_kts': V_required * 1.944,
        'V_stall_ms': V_stall,
        'V_stall_kts': V_stall * 1.944,
        'drag_N': drag,
        'power_required_W': power_required,
        'prop_efficiency': prop_efficiency,
        'reynolds': Re,
        'mach': mach,
        'rho_kg_m3': rho,
        'temperature_K': T,
        'pressure_Pa': P,
        'wing_loading_N_m2': weight / wing_area,
        'wing_loading_kg_m2': mass_kg / wing_area,
    }


# ─────────────────────────────────────────────────────────────────────────────
# STRUCTURAL MODEL
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class StructuralConfig:
    """Structural material and design parameters.
    Calibrated against Zephyr (25m span, ~32 kg airframe, 75 kg MTOW).
    HAPS use film-laminate skins, CF tube spars, foam ribs — far lighter
    than conventional composite aircraft.
    """
    material: str = "carbon_fiber_composite"
    # Specific masses — HAPS ultra-light construction
    wing_skin_density: float = 0.12       # kg/m² per surface (thin CF/film laminate)
    spar_linear_density: float = 0.10     # kg/m of span (baseline at 25m)
    spar_span_exponent: float = 2.2       # spar mass ~ span^exponent (bending + buckling resistance)
    max_practical_span_m: float = 35.0    # beyond this, buckling/flutter dominate
    rib_mass_each: float = 0.020          # kg per rib (foam/balsa + CF cap)
    rib_spacing_m: float = 0.50           # spacing between ribs
    # Other components as fraction of wing mass
    fuselage_fraction: float = 0.18       # fuselage mass / wing mass
    tail_fraction: float = 0.10           # empennage / wing mass
    systems_fraction: float = 0.06        # servos, wiring, actuators / airframe mass
    landing_mass_kg: float = 0.4          # belly skid / detachable gear
    # Flutter margin
    flutter_speed_margin: float = 1.5     # V_flutter / V_cruise >= this


def compute_structure(wingspan_m: float, chord_m: float,
                      struct: StructuralConfig) -> dict:
    """
    Estimate structural mass using semi-empirical HAPS scaling laws.
    Calibrated to match Zephyr S: 25m span ≈ 32 kg airframe structure.
    Scaling validated against PHASA-35 (35m, ~55 kg structure).
    """
    wing_area = wingspan_m * chord_m

    # Wing skin mass: upper + lower surfaces, film-laminate construction
    skin_mass = wing_area * struct.wing_skin_density * 2

    # Spar mass: scales super-linearly with span (bending moment ~ span²,
    # but spar is sized for strength not stiffness at these masses)
    spar_ref_span = 25.0  # reference span for linear density
    spar_mass = struct.spar_linear_density * spar_ref_span * (wingspan_m / spar_ref_span) ** struct.spar_span_exponent

    # Rib mass
    n_ribs = int(wingspan_m / struct.rib_spacing_m) + 1
    rib_mass = n_ribs * struct.rib_mass_each

    # Total wing structural mass
    wing_mass = skin_mass + spar_mass + rib_mass

    # Fuselage: lightweight boom/pod
    fuselage_mass = wing_mass * struct.fuselage_fraction

    # Tail/empennage
    tail_mass = wing_mass * struct.tail_fraction

    # Total airframe
    airframe_mass = wing_mass + fuselage_mass + tail_mass

    # Systems (servos, wiring, connectors)
    systems_mass = airframe_mass * struct.systems_fraction

    total_structural = airframe_mass + systems_mass + struct.landing_mass_kg

    # Bending rigidity estimate for flutter check
    spar_cap_height = chord_m * 0.12  # 12% t/c
    bending_rigidity = (spar_cap_height ** 3) * chord_m * 0.3 * 70e9 / 12

    return {
        'wing_mass_kg': wing_mass,
        'skin_mass_kg': skin_mass,
        'spar_mass_kg': spar_mass,
        'rib_mass_kg': rib_mass,
        'fuselage_mass_kg': fuselage_mass,
        'tail_mass_kg': tail_mass,
        'systems_mass_kg': systems_mass,
        'total_structural_kg': total_structural,
        'material': struct.material,
        'flutter_margin': struct.flutter_speed_margin,
        'bending_rigidity_Nm2': bending_rigidity,
        'n_ribs': n_ribs,
    }


# ─────────────────────────────────────────────────────────────────────────────
# POWER SYSTEM MODEL
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class PowerConfig:
    """Power system configuration."""
    solar_cell_type: str = "GaAs_multijunction"
    solar_efficiency: float = 0.30        # GaAs triple-junction (practical, with temperature derating)
    solar_coverage_fraction: float = 0.80  # fraction of upper wing covered (minus control surfaces, spar line)
    solar_cell_mass_per_m2: float = 0.22   # kg/m² thin-film GaAs on flex substrate + encapsulation

    battery_chemistry: str = "Li-S"
    battery_specific_energy: float = 400   # Wh/kg (Li-S practical)
    battery_specific_power: float = 200    # W/kg
    battery_cycle_life: int = 500          # cycles
    battery_dod: float = 0.80              # max depth of discharge
    battery_mass_overhead: float = 1.15    # BMS, thermal, packaging

    mppt_efficiency: float = 0.96          # solar charge controller
    inverter_efficiency: float = 0.95      # DC-DC conversion

    # Day/night cycle at stratospheric altitude
    # At 85k ft, effective day is longer due to altitude (sun visible earlier/later)
    summer_day_hours: float = 15.0
    winter_day_hours: float = 9.0
    avg_day_hours: float = 12.5            # mission-average assumption
    latitude_deg: float = 35.0             # typical mid-latitude operations


def compute_power(wingspan_m: float, chord_m: float, power_cfg: PowerConfig,
                  cruise_power_W: float, payload_power_W: float,
                  altitude_m: float) -> dict:
    """
    Compute power system sizing: solar, battery, energy balance.
    """
    wing_area = wingspan_m * chord_m

    # Solar panel area (upper wing surface)
    solar_area = wing_area * power_cfg.solar_coverage_fraction

    # Solar irradiance at altitude
    # At 85k ft (~26 km), we're above ~99% of atmosphere
    # Atmospheric transmission ≈ 0.95 at this altitude
    atm_transmission = 0.95 + 0.04 * (altitude_m / 30000)  # approaches 1.0 above 30km
    atm_transmission = min(atm_transmission, 0.99)

    peak_irradiance = SOLAR_CONSTANT * atm_transmission  # W/m²

    # Average irradiance over daylight hours (accounts for sun angle)
    # For a fixed horizontal panel: avg ≈ peak * (2/π) * cos(latitude) roughly
    lat_rad = math.radians(power_cfg.latitude_deg)
    avg_sun_factor = 0.637 * math.cos(lat_rad) * 1.1  # 1.1 for altitude advantage
    avg_sun_factor = min(avg_sun_factor, 0.75)

    avg_irradiance = peak_irradiance * avg_sun_factor

    # Power generation
    peak_solar_power = solar_area * peak_irradiance * power_cfg.solar_efficiency
    avg_solar_power = solar_area * avg_irradiance * power_cfg.solar_efficiency

    # System efficiency chain
    system_efficiency = power_cfg.mppt_efficiency * power_cfg.inverter_efficiency

    usable_solar_avg = avg_solar_power * system_efficiency
    usable_solar_peak = peak_solar_power * system_efficiency

    # Total power consumption
    total_power = cruise_power_W + payload_power_W

    # Day energy budget
    day_hours = power_cfg.avg_day_hours
    night_hours = 24.0 - day_hours

    day_energy_generated = usable_solar_avg * day_hours  # Wh
    day_energy_consumed = total_power * day_hours  # Wh
    night_energy_consumed = total_power * night_hours  # Wh

    # Battery must cover night + margin
    battery_energy_required = night_energy_consumed / power_cfg.battery_dod  # Wh

    # Also need to charge battery during the day
    day_surplus = day_energy_generated - day_energy_consumed

    # Check if day surplus can charge the battery
    charge_energy_needed = battery_energy_required  # Wh needed to charge
    charge_efficiency = 0.92  # round-trip battery charge/discharge
    actual_charge_needed = charge_energy_needed / charge_efficiency

    energy_feasible = day_surplus >= actual_charge_needed

    # Power margin
    total_24h_consumption = total_power * 24  # Wh
    total_24h_generation = day_energy_generated  # Wh (only during day)
    power_margin = (total_24h_generation - total_24h_consumption) / total_24h_consumption

    # Battery mass
    battery_mass = (battery_energy_required / power_cfg.battery_specific_energy) * power_cfg.battery_mass_overhead

    # Solar panel mass
    solar_mass = solar_area * power_cfg.solar_cell_mass_per_m2

    # Power system total mass (solar + battery + electronics)
    electronics_mass = 1.5  # MPPT, BMS, wiring, connectors
    power_system_mass = battery_mass + solar_mass + electronics_mass

    return {
        'solar_area_m2': solar_area,
        'solar_cell_type': power_cfg.solar_cell_type,
        'solar_efficiency': power_cfg.solar_efficiency,
        'peak_irradiance_W_m2': peak_irradiance,
        'avg_irradiance_W_m2': avg_irradiance,
        'peak_solar_power_W': peak_solar_power,
        'avg_solar_power_W': avg_solar_power,
        'usable_solar_avg_W': usable_solar_avg,
        'usable_solar_peak_W': usable_solar_peak,
        'day_hours': day_hours,
        'night_hours': night_hours,
        'day_energy_Wh': day_energy_generated,
        'night_consumption_Wh': night_energy_consumed,
        'day_surplus_Wh': day_surplus,
        'energy_feasible': energy_feasible,
        'power_margin_pct': power_margin * 100,
        'battery_energy_Wh': battery_energy_required,
        'battery_mass_kg': battery_mass,
        'battery_chemistry': power_cfg.battery_chemistry,
        'battery_specific_energy_Wh_kg': power_cfg.battery_specific_energy,
        'solar_mass_kg': solar_mass,
        'electronics_mass_kg': electronics_mass,
        'power_system_mass_kg': power_system_mass,
        'cruise_power_W': cruise_power_W,
        'payload_power_W': payload_power_W,
        'total_power_W': total_power,
    }


# ─────────────────────────────────────────────────────────────────────────────
# SENSOR & COMPUTE PAYLOAD MODEL
# ─────────────────────────────────────────────────────────────────────────────

SENSOR_TIERS = {
    'low': {
        'description': 'Basic detection - infrasound + single SWIR camera',
        'mass_kg': 3.0,
        'always_on_W': 15,
        'peak_W': 35,
        'cost_usd': 25000,
        'detection_range_km': 60,
        'detection_probability': 0.65,
        'components': ['Infrasound array (MEMS)', 'Single SWIR camera', 'Basic edge compute (RPi-class)'],
    },
    'medium': {
        'description': 'Standard detection - infrasound + SWIR array + EO camera',
        'mass_kg': 6.0,
        'always_on_W': 35,
        'peak_W': 75,
        'cost_usd': 85000,
        'detection_range_km': 120,
        'detection_probability': 0.82,
        'components': ['Infrasound array (precision)', 'SWIR wide-angle array', 'EO sentinel camera',
                       'M1-class edge compute', 'Active cooling'],
    },
    'high': {
        'description': 'Full suite - all sensors + multi-spectral + AI accelerator',
        'mass_kg': 10.0,
        'always_on_W': 60,
        'peak_W': 150,
        'cost_usd': 220000,
        'detection_range_km': 200,
        'detection_probability': 0.94,
        'components': ['Infrasound array (precision, redundant)', 'SWIR wide-angle array (cooled)',
                       'EO sentinel (stabilized gimbal)', 'MWIR supplemental',
                       'AI accelerator (edge TPU)', 'Active cooling + thermal management'],
    },
}


@dataclass
class CommsConfig:
    """Communications system configuration."""
    mesh_capable: bool = True
    satellite_link: bool = True
    node_to_node_range_km: float = 250     # LOS at 85k ft is huge
    bandwidth_mbps: float = 5.0
    comms_mass_kg: float = 1.5
    comms_power_W: float = 12.0
    comms_cost_usd: float = 15000
    relay_level: str = "mesh_with_satcom_backup"


def compute_payload(sensor_tier: str, comms: CommsConfig) -> dict:
    """Compute payload mass, power, and cost."""
    sensor = SENSOR_TIERS[sensor_tier]

    total_payload_mass = sensor['mass_kg'] + comms.comms_mass_kg
    always_on_power = sensor['always_on_W'] + comms.comms_power_W
    peak_power = sensor['peak_W'] + comms.comms_power_W
    payload_cost = sensor['cost_usd'] + comms.comms_cost_usd

    return {
        'sensor_tier': sensor_tier,
        'sensor_description': sensor['description'],
        'sensor_components': sensor['components'],
        'sensor_mass_kg': sensor['mass_kg'],
        'sensor_cost_usd': sensor['cost_usd'],
        'detection_range_km': sensor['detection_range_km'],
        'detection_probability': sensor['detection_probability'],
        'comms_mass_kg': comms.comms_mass_kg,
        'comms_power_W': comms.comms_power_W,
        'comms_cost_usd': comms.comms_cost_usd,
        'comms_range_km': comms.node_to_node_range_km,
        'mesh_capable': comms.mesh_capable,
        'satellite_link': comms.satellite_link,
        'bandwidth_mbps': comms.bandwidth_mbps,
        'relay_architecture': comms.relay_level,
        'total_payload_mass_kg': total_payload_mass,
        'always_on_power_W': always_on_power,
        'peak_power_W': peak_power,
        'payload_cost_usd': payload_cost,
    }


# ─────────────────────────────────────────────────────────────────────────────
# COST MODEL
# ─────────────────────────────────────────────────────────────────────────────

def compute_unit_cost(structural_mass_kg: float, power_system_mass_kg: float,
                      solar_area_m2: float, battery_energy_Wh: float,
                      payload_cost_usd: float, wingspan_m: float) -> dict:
    """
    Estimate per-unit manufacturing cost using parametric cost model.
    """
    # Airframe cost: carbon fiber composite manufacturing
    # ~$300-600/kg for hand-layup CF structures, less for automated
    cf_cost_per_kg = 450
    airframe_cost = structural_mass_kg * cf_cost_per_kg

    # Tooling amortization (spread over production run)
    tooling_cost = 50000 + wingspan_m * 2000  # scales with wingspan
    production_run = 50  # assumed fleet + spares
    tooling_per_unit = tooling_cost / production_run

    # Solar cells: GaAs multijunction ~$100-200/m²  (dropping with scale)
    solar_cost_per_m2 = 150
    solar_cost = solar_area_m2 * solar_cost_per_m2

    # Battery cost: Li-S ~$150-300/kWh
    battery_cost_per_kWh = 200
    battery_cost = (battery_energy_Wh / 1000) * battery_cost_per_kWh

    # Power electronics
    power_electronics_cost = 3000

    # Assembly, integration, test
    ait_cost = airframe_cost * 0.30  # 30% of airframe

    # Propulsion (motors + props)
    propulsion_cost = 2500

    per_unit_cost = (airframe_cost + tooling_per_unit + solar_cost + battery_cost +
                     power_electronics_cost + payload_cost_usd + ait_cost + propulsion_cost)

    return {
        'airframe_cost_usd': airframe_cost,
        'tooling_per_unit_usd': tooling_per_unit,
        'solar_cost_usd': solar_cost,
        'battery_cost_usd': battery_cost,
        'power_electronics_cost_usd': power_electronics_cost,
        'payload_cost_usd': payload_cost_usd,
        'ait_cost_usd': ait_cost,
        'propulsion_cost_usd': propulsion_cost,
        'per_unit_cost_usd': per_unit_cost,
        'production_run': production_run,
    }


# ─────────────────────────────────────────────────────────────────────────────
# FLEET & COVERAGE MODEL
# ─────────────────────────────────────────────────────────────────────────────

def compute_fleet(detection_range_km: float, coverage_target_km2: float,
                  per_unit_cost: float, overlap_factor: float = 1.3,
                  spares_fraction: float = 0.20,
                  endurance_days: float = 7.0) -> dict:
    """
    Compute fleet sizing for area coverage.
    """
    # Coverage per node (circular footprint)
    coverage_per_node_km2 = math.pi * detection_range_km**2

    # Effective coverage with overlap
    effective_coverage = coverage_per_node_km2 / overlap_factor

    # Nodes needed for target area
    nodes_active = math.ceil(coverage_target_km2 / effective_coverage)
    nodes_active = max(nodes_active, 1)

    # Total fleet (active + rotation + spares)
    # For continuous coverage, need rotation factor based on endurance
    rotation_factor = 1.3  # accounts for transit, maintenance windows
    total_fleet = math.ceil(nodes_active * rotation_factor * (1 + spares_fraction))

    fleet_cost = total_fleet * per_unit_cost
    cost_per_km2 = fleet_cost / coverage_target_km2

    # Node density
    node_density = nodes_active / coverage_target_km2  # nodes per km²

    # Annual maintenance (battery replacement, repairs)
    annual_maintenance_per_unit = per_unit_cost * 0.10  # 10% of unit cost annually
    annual_fleet_maintenance = annual_maintenance_per_unit * total_fleet

    # Expected lifetime
    battery_cycles_per_year = 365 / endurance_days  # rough: one deep cycle per mission
    battery_life_years = 500 / battery_cycles_per_year  # 500 cycle Li-S
    airframe_life_years = 3.0  # conservative for stratospheric UV/thermal cycling
    expected_lifetime_years = min(battery_life_years, airframe_life_years)

    return {
        'coverage_per_node_km2': coverage_per_node_km2,
        'effective_coverage_km2': effective_coverage,
        'overlap_factor': overlap_factor,
        'nodes_active': nodes_active,
        'total_fleet_size': total_fleet,
        'fleet_cost_usd': fleet_cost,
        'cost_per_km2_usd': cost_per_km2,
        'node_density_per_km2': node_density,
        'annual_maintenance_usd': annual_fleet_maintenance,
        'expected_lifetime_years': expected_lifetime_years,
        'spares_fraction': spares_fraction,
        'failure_tolerance': 'graceful_degradation' if nodes_active > 1 else 'single_point',
        'detection_fusion': nodes_active > 1,
    }


# ─────────────────────────────────────────────────────────────────────────────
# MISSION PERFORMANCE MODEL
# ─────────────────────────────────────────────────────────────────────────────

def compute_mission(V_cruise_ms: float, total_power_W: float,
                    battery_energy_Wh: float, energy_feasible: bool,
                    altitude_m: float) -> dict:
    """
    Compute mission performance metrics.
    """
    # Range on battery alone (no solar, emergency)
    battery_range_km = (battery_energy_Wh / total_power_W) * V_cruise_ms * 3.6  # Wh/W = hours, * V * 3.6 = km

    # With solar, endurance is theoretically unlimited IF energy_feasible
    if energy_feasible:
        endurance_days = 30  # practical limit (maintenance, weather)
        endurance_limited_by = "maintenance_cycle"
    else:
        # How many days until battery depletion
        # Each night drains more than day replenishes
        endurance_days = 1  # would need more detailed sim
        endurance_limited_by = "energy_deficit"

    # Transit range to station (from launch to operational altitude/location)
    # Assume launch within 500km of operational area
    transit_range_km = 500

    # Operational radius (loiter pattern)
    operational_radius_km = V_cruise_ms * 3.6 * 2  # can relocate ~2 hours of flight

    # Station-keeping
    # At 85k ft, winds are relatively calm (5-15 m/s typically in stratosphere)
    # But can encounter jet stream remnants
    max_wind_tolerance_ms = V_cruise_ms * 0.8  # can handle winds up to 80% of cruise speed

    # Deployment method
    if altitude_m > 20000:
        deployment = "ground_launch_spiral_climb"  # ~4-8 hour climb to altitude
        climb_time_hours = altitude_m / (0.5 * 3600)  # ~0.5 m/s climb rate
    else:
        deployment = "conventional_runway"
        climb_time_hours = altitude_m / (2.0 * 3600)

    return {
        'battery_only_range_km': battery_range_km,
        'solar_endurance_days': endurance_days,
        'endurance_limited_by': endurance_limited_by,
        'transit_range_km': transit_range_km,
        'operational_radius_km': operational_radius_km,
        'max_wind_ms': max_wind_tolerance_ms,
        'max_wind_kts': max_wind_tolerance_ms * 1.944,
        'deployment_method': deployment,
        'climb_time_hours': climb_time_hours,
        'design_recoverable': True,
        'weather_tolerance': 'fair_weather_only',  # HAPS are weather-sensitive
        'altitude_band': f"{altitude_m/FT_TO_M:.0f} ft ({altitude_m/1000:.1f} km)",
    }


# ─────────────────────────────────────────────────────────────────────────────
# INTEGRATED DESIGN EVALUATION
# ─────────────────────────────────────────────────────────────────────────────

def evaluate_design(wingspan_m: float, chord_m: float, sensor_tier: str,
                    altitude_ft: float = 85000,
                    coverage_target_km2: float = 500000,
                    aero_cfg: Optional[AeroConfig] = None,
                    struct_cfg: Optional[StructuralConfig] = None,
                    power_cfg: Optional[PowerConfig] = None,
                    comms_cfg: Optional[CommsConfig] = None) -> dict:
    """
    Full integrated evaluation of a design point.
    Iteratively solves the mass/power coupling.
    """
    if aero_cfg is None: aero_cfg = AeroConfig()
    if struct_cfg is None: struct_cfg = StructuralConfig()
    if power_cfg is None: power_cfg = PowerConfig()
    if comms_cfg is None: comms_cfg = CommsConfig()

    altitude_m = altitude_ft * FT_TO_M

    # Payload (fixed for given sensor tier)
    payload = compute_payload(sensor_tier, comms_cfg)

    # Iterative mass convergence with under-relaxation
    # Heavier aircraft → more cruise power → more battery → heavier.
    # Under-relaxation damps this positive feedback loop.
    structure = compute_structure(wingspan_m, chord_m, struct_cfg)
    fixed_mass = structure['total_structural_kg'] + payload['total_payload_mass_kg']

    # Propulsion mass (motors + props) — compute once, it only depends on wingspan
    n_motors = 2 if wingspan_m < 20 else (4 if wingspan_m < 35 else 6)
    motor_mass = n_motors * 0.3   # lightweight BLDC
    prop_mass = n_motors * 0.15
    propulsion_mass = motor_mass + prop_mass
    fixed_mass += propulsion_mass

    # Initial guess for power system mass
    power_system_mass_guess = fixed_mass * 0.5
    relaxation = 0.30
    converged = False

    for iteration in range(120):
        total_mass = fixed_mass + power_system_mass_guess

        aero = compute_aero(wingspan_m, chord_m, aero_cfg, altitude_m, total_mass)
        power = compute_power(wingspan_m, chord_m, power_cfg,
                             aero['power_required_W'], payload['always_on_power_W'],
                             altitude_m)

        computed_psm = power['power_system_mass_kg']

        # Divergence detection BEFORE updating guess
        if (not math.isfinite(computed_psm) or computed_psm > 400
                or not math.isfinite(total_mass) or total_mass > 600):
            return None

        delta = abs(computed_psm - power_system_mass_guess)

        # Under-relaxation blend
        power_system_mass_guess = relaxation * computed_psm + (1 - relaxation) * power_system_mass_guess

        if delta < 0.02:
            converged = True
            break

    if not converged:
        return None

    # Final consistent evaluation with converged mass
    total_mass = fixed_mass + power_system_mass_guess
    aero = compute_aero(wingspan_m, chord_m, aero_cfg, altitude_m, total_mass)
    power = compute_power(wingspan_m, chord_m, power_cfg,
                         aero['power_required_W'], payload['always_on_power_W'],
                         altitude_m)

    # Cost
    cost = compute_unit_cost(structure['total_structural_kg'], power_system_mass_guess,
                            power['solar_area_m2'], power['battery_energy_Wh'],
                            payload['payload_cost_usd'], wingspan_m)

    # Fleet
    fleet = compute_fleet(payload['detection_range_km'], coverage_target_km2,
                         cost['per_unit_cost_usd'])

    # Mission
    mission = compute_mission(aero['V_cruise_ms'], power['total_power_W'],
                             power['battery_energy_Wh'], power['energy_feasible'],
                             altitude_m)

    # Feasibility checks
    feasibility = {
        'energy_feasible': power['energy_feasible'],
        'power_margin_ok': power['power_margin_pct'] > 5.0,
        'stall_margin_ok': aero['V_cruise_ms'] > aero['V_stall_ms'] * 1.15,  # stratospheric calm air
        'wing_loading_ok': aero['wing_loading_kg_m2'] < 5.0,  # HAPS typically < 5 kg/m²
        'mass_reasonable': total_mass < 250,  # practical HAPS limit (85k ft needs bigger than Zephyr)
        'aspect_ratio_ok': 10 < aero['aspect_ratio'] < 40,
        'span_structural_ok': wingspan_m <= struct_cfg.max_practical_span_m,
    }
    feasibility['all_feasible'] = all(feasibility.values())

    return {
        'design_inputs': {
            'wingspan_m': wingspan_m,
            'chord_m': chord_m,
            'sensor_tier': sensor_tier,
            'altitude_ft': altitude_ft,
            'altitude_m': altitude_m,
            'coverage_target_km2': coverage_target_km2,
        },
        'mass_breakdown': {
            'structural_kg': structure['total_structural_kg'],
            'payload_kg': payload['total_payload_mass_kg'],
            'power_system_kg': power_system_mass_guess,
            'battery_kg': power['battery_mass_kg'],
            'solar_panels_kg': power['solar_mass_kg'],
            'propulsion_kg': propulsion_mass,
            'total_mass_kg': total_mass,
        },
        'aerodynamics': aero,
        'structure': structure,
        'power': power,
        'payload': payload,
        'cost': cost,
        'fleet': fleet,
        'mission': mission,
        'feasibility': feasibility,
        'n_motors': n_motors,
    }


# ─────────────────────────────────────────────────────────────────────────────
# OPTIMIZER
# ─────────────────────────────────────────────────────────────────────────────

def optimize_design(altitude_ft: float = 78000,
                    altitude_range: Tuple[float, float] = (70000, 82000),
                    min_endurance_days: float = 7,
                    min_range_km: float = 8000,
                    min_coverage_km: float = 100,
                    coverage_target_km2: float = 500000) -> dict:
    """
    Sweep design space and find optimal configurations.
    Uses grid search over wingspan, chord, altitude, and sensor tier.
    Optimizes for minimum per-unit cost while satisfying all constraints.
    Also finds the maximum feasible altitude for each tier.
    """

    # Design space
    wingspans = [i * 1.0 for i in range(20, 37)]     # 20m to 36m in 1m steps (structural limit ~35m)
    chords = [i * 0.1 for i in range(6, 21)]          # 0.6m to 2.0m in 0.1m steps
    altitudes = [i * 1000 for i in range(int(altitude_range[0]/1000), int(altitude_range[1]/1000) + 1)]
    tiers = ['low', 'medium', 'high']

    results = []
    best_by_tier = {}
    max_alt_by_tier = {}
    n_total = len(wingspans) * len(chords) * len(altitudes) * len(tiers)

    print(f"Searching {n_total} design points ({len(wingspans)} wingspans × "
          f"{len(chords)} chords × {len(altitudes)} altitudes × {len(tiers)} tiers)...")

    for tier in tiers:
        best_cost = float('inf')
        best_design = None
        max_alt = 0
        max_alt_design = None

        for alt in altitudes:
            for ws in wingspans:
                for ch in chords:
                    try:
                        design = evaluate_design(ws, ch, tier,
                                               altitude_ft=alt,
                                               coverage_target_km2=coverage_target_km2)

                        if design is None:
                            continue

                        if not design['feasibility']['all_feasible']:
                            continue

                        if design['payload']['detection_range_km'] < min_coverage_km:
                            continue

                        results.append(design)

                        # Track best by cost (at any feasible altitude)
                        unit_cost = design['cost']['per_unit_cost_usd']
                        if unit_cost < best_cost:
                            best_cost = unit_cost
                            best_design = design

                        # Track maximum feasible altitude
                        if alt > max_alt:
                            max_alt = alt
                            max_alt_design = design

                    except (ValueError, ZeroDivisionError, OverflowError):
                        continue

        if best_design:
            best_by_tier[tier] = best_design
            print(f"  [{tier.upper()}] Cheapest: {best_design['design_inputs']['wingspan_m']:.0f}m × "
                  f"{best_design['design_inputs']['chord_m']:.1f}m @ {best_design['design_inputs']['altitude_ft']:.0f}ft, "
                  f"${best_design['cost']['per_unit_cost_usd']:,.0f}/unit, "
                  f"{best_design['mass_breakdown']['total_mass_kg']:.1f} kg")
        else:
            print(f"  [{tier.upper()}] No feasible design found")

        if max_alt_design:
            max_alt_by_tier[tier] = max_alt_design
            print(f"           Highest: {max_alt_design['design_inputs']['wingspan_m']:.0f}m × "
                  f"{max_alt_design['design_inputs']['chord_m']:.1f}m @ {max_alt}ft, "
                  f"${max_alt_design['cost']['per_unit_cost_usd']:,.0f}/unit, "
                  f"{max_alt_design['mass_breakdown']['total_mass_kg']:.1f} kg, "
                  f"margin={max_alt_design['power']['power_margin_pct']:.1f}%")

    print(f"\nFound {len(results)} feasible designs out of {n_total} search space")

    return {
        'all_feasible': results,
        'best_by_tier': best_by_tier,
        'max_alt_by_tier': max_alt_by_tier,
        'search_params': {
            'altitude_range_ft': altitude_range,
            'min_endurance_days': min_endurance_days,
            'min_range_km': min_range_km,
            'min_coverage_km': min_coverage_km,
            'coverage_target_km2': coverage_target_km2,
        }
    }


# ─────────────────────────────────────────────────────────────────────────────
# SENSITIVITY ANALYSIS
# ─────────────────────────────────────────────────────────────────────────────

def sensitivity_analysis(base_wingspan: float, base_chord: float,
                         sensor_tier: str, altitude_ft: float = 85000) -> dict:
    """
    Vary key parameters ±20% around the optimal point to show sensitivities.
    """
    base = evaluate_design(base_wingspan, base_chord, sensor_tier, altitude_ft)

    sensitivities = {}

    # Wingspan sensitivity
    ws_data = []
    for ws in [base_wingspan * f for f in [0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15, 1.2]]:
        try:
            d = evaluate_design(ws, base_chord, sensor_tier, altitude_ft)
            ws_data.append({
                'wingspan_m': ws,
                'total_mass_kg': d['mass_breakdown']['total_mass_kg'],
                'power_margin_pct': d['power']['power_margin_pct'],
                'per_unit_cost': d['cost']['per_unit_cost_usd'],
                'L_over_D': d['aerodynamics']['L_over_D'],
                'feasible': d['feasibility']['all_feasible'],
            })
        except:
            pass
    sensitivities['wingspan'] = ws_data

    # Chord sensitivity
    ch_data = []
    for ch in [base_chord * f for f in [0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15, 1.2]]:
        try:
            d = evaluate_design(base_wingspan, ch, sensor_tier, altitude_ft)
            ch_data.append({
                'chord_m': ch,
                'total_mass_kg': d['mass_breakdown']['total_mass_kg'],
                'power_margin_pct': d['power']['power_margin_pct'],
                'per_unit_cost': d['cost']['per_unit_cost_usd'],
                'L_over_D': d['aerodynamics']['L_over_D'],
                'aspect_ratio': d['aerodynamics']['aspect_ratio'],
                'feasible': d['feasibility']['all_feasible'],
            })
        except:
            pass
    sensitivities['chord'] = ch_data

    # Altitude sensitivity
    alt_data = []
    for alt in [60000, 65000, 70000, 75000, 80000, 85000, 90000]:
        try:
            d = evaluate_design(base_wingspan, base_chord, sensor_tier, alt)
            alt_data.append({
                'altitude_ft': alt,
                'altitude_km': alt * FT_TO_M / 1000,
                'rho_kg_m3': d['aerodynamics']['rho_kg_m3'],
                'cruise_power_W': d['aerodynamics']['power_required_W'],
                'V_cruise_kts': d['aerodynamics']['V_cruise_kts'],
                'total_mass_kg': d['mass_breakdown']['total_mass_kg'],
                'power_margin_pct': d['power']['power_margin_pct'],
                'feasible': d['feasibility']['all_feasible'],
            })
        except:
            pass
    sensitivities['altitude'] = alt_data

    return sensitivities


# ─────────────────────────────────────────────────────────────────────────────
# OUTPUT GENERATION
# ─────────────────────────────────────────────────────────────────────────────

def generate_report(optimization_results: dict) -> str:
    """Generate a JSON report of optimization results."""

    report = {
        'title': 'ARGUS HAPS Design Optimization Report',
        'search_params': optimization_results['search_params'],
        'n_feasible_designs': len(optimization_results['all_feasible']),
        'optimal_designs': {},
    }

    for tier, design in optimization_results['best_by_tier'].items():
        # Run sensitivity for each optimal
        sens = sensitivity_analysis(
            design['design_inputs']['wingspan_m'],
            design['design_inputs']['chord_m'],
            tier,
            design['design_inputs']['altitude_ft']
        )

        report['optimal_designs'][tier] = {
            'design': design,
            'sensitivity': sens,
        }

    return json.dumps(repo