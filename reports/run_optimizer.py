#!/usr/bin/env python3
"""
ARGUS Optimizer Runner — runs the optimization and prints results.
"""
import json
from argus_engine import optimize_design, generate_report


def print_design(d, label=""):
    """Pretty-print a single design point."""
    if label:
        print(f"\n{'=' * 60}")
        print(f"  {label}")
        print(f"{'=' * 60}")

    inp = d['design_inputs']
    mb = d['mass_breakdown']
    aero = d['aerodynamics']
    pwr = d['power']
    pay = d['payload']
    cst = d['cost']
    flt = d['fleet']
    mis = d['mission']
    feas = d['feasibility']

    print(f"\n  AIRFRAME")
    print(f"     Altitude:        {inp['altitude_ft']:.0f} ft ({inp['altitude_m']/1000:.1f} km)")
    print(f"     Wingspan:        {inp['wingspan_m']:.1f} m")
    print(f"     Chord:           {inp['chord_m']:.2f} m")
    print(f"     Wing Area:       {aero['wing_area_m2']:.1f} m2")
    print(f"     Aspect Ratio:    {aero['aspect_ratio']:.1f}")
    print(f"     Total Mass:      {mb['total_mass_kg']:.1f} kg")
    print(f"     Wing Loading:    {aero['wing_loading_kg_m2']:.2f} kg/m2")

    print(f"\n  POWER SYSTEM")
    print(f"     Solar Area:      {pwr['solar_area_m2']:.1f} m2 ({pwr['solar_cell_type']})")
    print(f"     Peak Solar:      {pwr['peak_solar_power_W']:.0f} W")
    print(f"     Avg Usable:      {pwr['usable_solar_avg_W']:.0f} W")
    print(f"     Battery:         {pwr['battery_energy_Wh']:.0f} Wh ({pwr['battery_chemistry']})")
    print(f"     Battery Mass:    {pwr['battery_mass_kg']:.1f} kg")
    print(f"     Power Margin:    {pwr['power_margin_pct']:.1f}%")

    print(f"\n  FLIGHT PERFORMANCE")
    print(f"     Cruise Speed:    {aero['V_cruise_kts']:.1f} kts")
    print(f"     Stall Speed:     {aero['V_stall_kts']:.1f} kts")
    print(f"     L/D Ratio:       {aero['L_over_D']:.1f} (best: {aero['LD_best']:.1f})")
    print(f"     Cruise Power:    {aero['power_required_W']:.0f} W")
    print(f"     Mach:            {aero['mach']:.3f}")
    print(f"     Endurance:       {mis['solar_endurance_days']} days")

    print(f"\n  SENSORS")
    print(f"     Tier:            {pay['sensor_tier'].upper()} - {pay['sensor_description']}")
    print(f"     Detection Range: {pay['detection_range_km']} km")
    print(f"     Det Probability: {pay['detection_probability']:.0%}")
    print(f"     Always-On:       {pay['always_on_power_W']} W | Mass: {pay['sensor_mass_kg']:.1f} kg")

    print(f"\n  COMMS")
    print(f"     Range: {pay['comms_range_km']:.0f} km | Mesh: {'Y' if pay['mesh_capable'] else 'N'} | Sat: {'Y' if pay['satellite_link'] else 'N'}")

    print(f"\n  COST")
    print(f"     Per-Unit:  ${cst['per_unit_cost_usd']:,.0f}")
    print(f"     Breakdown: airframe ${cst['airframe_cost_usd']:,.0f} | solar ${cst['solar_cost_usd']:,.0f} | battery ${cst['battery_cost_usd']:,.0f} | payload ${cst['payload_cost_usd']:,.0f}")

    spd = flt.get('swarm_detection_pd', pay['detection_probability'])
    snpd = flt.get('single_node_pd', pay['detection_probability'])
    novl = flt.get('avg_overlap_nodes', 1)
    print(f"\n  SWARM DETECTION")
    print(f"     Single-node Pd:  {snpd:.0%}")
    print(f"     Avg overlap:     {novl} nodes")
    print(f"     Fused swarm Pd:  {spd:.0%}")

    print(f"\n  FLEET ({inp['coverage_target_km2']:,} km2)")
    print(f"     Nodes: {flt['nodes_active']} active / {flt['total_fleet_size']} total")
    print(f"     Fleet cost: ${flt['fleet_cost_usd']:,.0f} (${flt['cost_per_km2_usd']:.2f}/km2)")
    print(f"     Coverage/node: {flt['coverage_per_node_km2']:,.0f} km2 | Lifetime: {flt['expected_lifetime_years']:.1f} yr")

    print(f"\n  FEASIBILITY")
    for check, passed in feas.items():
        if check != 'all_feasible':
            print(f"     [{'PASS' if passed else 'FAIL'}] {check}")


if __name__ == "__main__":
    print("=" * 70)
    print("  ARGUS HAPS Design Optimizer")
    print("  High-Altitude Platform Station for Hypersonic Missile Detection")
    print("=" * 70)

    results = optimize_design(
        altitude_range=(55000, 75000),
        min_endurance_days=7,
        min_range_km=8000,
        min_coverage_km=25,       # low per-node OK, swarm compensates
        coverage_target_km2=500000,
    )

    # Save JSON report
    report_json = generate_report(results)
    with open('argus_optimization_report.json', 'w') as f:
        f.write(report_json)

    # Print cheapest designs
    print("\n" + "=" * 70)
    print("  CHEAPEST FEASIBLE DESIGNS (by sensor tier)")
    print("=" * 70)
    for tier, design in results['best_by_tier'].items():
        print_design(design, f"TIER: {tier.upper()} - LOWEST COST")

    # Print max altitude designs
    # Print max altitude designs
    print("\n" + "=" * 70)
    print("  MAXIMUM ALTITUDE DESIGNS (by sensor tier)")
    print("=" * 70)
    for tier, design in results.get('max_alt_by_tier', {}).items():
        print_design(design, "TIER: " + tier.upper() + " - MAX ALTITUDE")

    n = len(results['all_feasible'])
    print("\n" + "=" * 70)
    print("  " + str(n) + " feasible designs found")
    print("  Report saved to: argus_optimization_report.json")
    print("=" * 70)
