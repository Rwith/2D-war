"""
Servo selection calculator for RC aircraft control surfaces.
Supports stabilators (all-moving tails) and conventional elevator/flap surfaces.
"""

import math

# ---------------------------------------------------------------------------
# Servo database  (name, torque_oz_in @ voltage, speed_sec_60deg @ voltage)
# ---------------------------------------------------------------------------
SERVO_DB = [
    {
        "name": "Hitec HS-5055MG",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 33.3, "kg_cm": 2.4},
            {"voltage_v": 6.0, "oz_in": 41.7, "kg_cm": 3.0},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.15},
            {"voltage_v": 6.0, "sec_60deg": 0.13},
        ],
        "weight_g": 19.8,
        "type": "analog",
        "category": "micro",
    },
    {
        "name": "Hitec HS-5070MH",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 47.2, "kg_cm": 3.4},
            {"voltage_v": 6.0, "oz_in": 68.0, "kg_cm": 4.9},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.14},
            {"voltage_v": 6.0, "sec_60deg": 0.12},
        ],
        "weight_g": 20.2,
        "type": "digital",
        "category": "micro",
    },
    {
        "name": "Savox SH-0257MG",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 43.0, "kg_cm": 3.1},
            {"voltage_v": 6.0, "oz_in": 56.9, "kg_cm": 4.1},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.13},
            {"voltage_v": 6.0, "sec_60deg": 0.11},
        ],
        "weight_g": 21.0,
        "type": "digital",
        "category": "micro",
    },
    {
        "name": "Hitec HS-5625MG",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 111.0, "kg_cm": 8.0},
            {"voltage_v": 6.0, "oz_in": 138.9, "kg_cm": 10.0},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.16},
            {"voltage_v": 6.0, "sec_60deg": 0.14},
        ],
        "weight_g": 55.2,
        "type": "digital",
        "category": "standard",
    },
    {
        "name": "Hitec HS-5585MH",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 180.0, "kg_cm": 13.0},
            {"voltage_v": 6.0, "oz_in": 208.3, "kg_cm": 15.0},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.18},
            {"voltage_v": 6.0, "sec_60deg": 0.15},
        ],
        "weight_g": 58.0,
        "type": "digital",
        "category": "standard",
    },
    {
        "name": "Savox SV-0220MG",
        "torque": [
            {"voltage_v": 6.0, "oz_in": 194.4, "kg_cm": 14.0},
            {"voltage_v": 7.4, "oz_in": 222.2, "kg_cm": 16.0},
        ],
        "speed": [
            {"voltage_v": 6.0, "sec_60deg": 0.13},
            {"voltage_v": 7.4, "sec_60deg": 0.11},
        ],
        "weight_g": 56.0,
        "type": "digital",
        "category": "standard",
    },
    {
        "name": "Hitec HS-5685MH",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 180.6, "kg_cm": 13.0},
            {"voltage_v": 6.0, "oz_in": 236.1, "kg_cm": 17.0},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.20},
            {"voltage_v": 6.0, "sec_60deg": 0.17},
        ],
        "weight_g": 60.0,
        "type": "digital",
        "category": "standard",
    },
    {
        "name": "Savox SC-1268SG",
        "torque": [
            {"voltage_v": 6.0, "oz_in": 250.0, "kg_cm": 18.0},
            {"voltage_v": 7.4, "oz_in": 305.6, "kg_cm": 22.0},
        ],
        "speed": [
            {"voltage_v": 6.0, "sec_60deg": 0.17},
            {"voltage_v": 7.4, "sec_60deg": 0.14},
        ],
        "weight_g": 68.0,
        "type": "digital",
        "category": "large",
    },
    {
        "name": "Futaba S3071SV",
        "torque": [
            {"voltage_v": 6.0, "oz_in": 277.8, "kg_cm": 20.0},
            {"voltage_v": 7.4, "oz_in": 375.0, "kg_cm": 27.0},
        ],
        "speed": [
            {"voltage_v": 6.0, "sec_60deg": 0.14},
            {"voltage_v": 7.4, "sec_60deg": 0.12},
        ],
        "weight_g": 68.3,
        "type": "digital",
        "category": "large",
    },
    {
        "name": "Hitec HS-7954SH",
        "torque": [
            {"voltage_v": 6.0, "oz_in": 333.3, "kg_cm": 24.0},
            {"voltage_v": 7.4, "oz_in": 430.6, "kg_cm": 31.0},
        ],
        "speed": [
            {"voltage_v": 6.0, "sec_60deg": 0.15},
            {"voltage_v": 7.4, "sec_60deg": 0.12},
        ],
        "weight_g": 96.0,
        "type": "digital",
        "category": "large",
    },
    {
        "name": "Savox SB-2274SG",
        "torque": [
            {"voltage_v": 6.0, "oz_in": 361.1, "kg_cm": 26.0},
            {"voltage_v": 8.4, "oz_in": 486.1, "kg_cm": 35.0},
        ],
        "speed": [
            {"voltage_v": 6.0, "sec_60deg": 0.17},
            {"voltage_v": 8.4, "sec_60deg": 0.13},
        ],
        "weight_g": 108.0,
        "type": "digital",
        "category": "large",
    },
]

AIR_DENSITY_KG_M3 = 1.225  # sea level, ISA
SAFETY_FACTOR = 2.0
AC_POSITION = 0.25          # aerodynamic centre as fraction of chord (thin airfoil)


def dynamic_pressure(speed_kmh: float) -> float:
    """Return dynamic pressure in Pa at given airspeed (km/h)."""
    v = speed_kmh / 3.6
    return 0.5 * AIR_DENSITY_KG_M3 * v ** 2


def mean_aerodynamic_chord(area_mm2: float, span_mm: float) -> float:
    """Return mean aerodynamic chord in mm."""
    return area_mm2 / span_mm


def hinge_moment_stabilator(
    area_mm2: float,
    span_mm: float,
    chord_mm: float,
    hinge_from_le_mm: float,
    top_speed_kmh: float,
    cl: float = 1.0,
) -> dict:
    """
    Calculate hinge moment for a full-flying stabilator.

    The servo is the pivot. Load comes from the offset between the pivot
    position and the aerodynamic centre (25% of chord).

    Parameters
    ----------
    area_mm2          : planform area of one half (mm²)
    span_mm           : span of one half (mm)
    chord_mm          : chord of the surface (front to back, mm)
    hinge_from_le_mm  : pivot distance from leading edge (mm)
    top_speed_kmh     : maximum airspeed (km/h)
    cl                : conservative lift coefficient at max deflection

    Returns
    -------
    dict with moment (N·m), required torque (oz-in), and geometry info
    """
    mac_mm = mean_aerodynamic_chord(area_mm2, span_mm)
    area_m2 = area_mm2 / 1e6

    # Pivot fraction and moment arm use the actual chord (not MAC)
    pivot_frac = hinge_from_le_mm / chord_mm
    ac_position_mm = AC_POSITION * chord_mm
    moment_arm_m = abs(ac_position_mm - hinge_from_le_mm) / 1000.0

    # Flag over-balance: if pivot is aft of AC the surface is over-balanced
    over_balanced = hinge_from_le_mm > ac_position_mm

    q = dynamic_pressure(top_speed_kmh)
    hinge_moment_nm = q * cl * area_m2 * moment_arm_m
    required_nm = hinge_moment_nm * SAFETY_FACTOR
    required_oz_in = required_nm * 141.612  # 1 N·m = 141.612 oz-in

    return {
        "surface_type": "stabilator",
        "chord_mm": round(chord_mm, 2),
        "mac_mm": round(mac_mm, 2),
        "pivot_fraction": round(pivot_frac * 100, 1),
        "ac_position_mm": round(ac_position_mm, 2),
        "ac_fraction": AC_POSITION * 100,
        "moment_arm_mm": round(moment_arm_m * 1000, 2),
        "over_balanced": over_balanced,
        "dynamic_pressure_pa": round(q, 1),
        "hinge_moment_nm": round(hinge_moment_nm, 3),
        "hinge_moment_oz_in": round(hinge_moment_nm * 141.612, 1),
        "required_torque_nm": round(required_nm, 3),
        "required_torque_oz_in": round(required_oz_in, 1),
        "safety_factor": SAFETY_FACTOR,
    }


def hinge_moment_elevator(
    area_mm2: float,
    span_mm: float,
    chord_mm: float,
    hinge_from_le_mm: float,
    top_speed_kmh: float,
    ch_eff: float = 0.04,
) -> dict:
    """
    Calculate hinge moment for a conventional plain-flap elevator.

    Parameters
    ----------
    area_mm2          : planform area of the elevator surface (mm²)
    span_mm           : elevator span (mm)
    chord_mm          : chord of the surface (front to back, mm)
    hinge_from_le_mm  : hinge distance from leading edge of elevator (mm)
    top_speed_kmh     : maximum airspeed (km/h)
    ch_eff            : effective hinge moment coefficient (accounts for balance)

    Returns
    -------
    dict with moment, required torque, and geometry info
    """
    mac_mm = mean_aerodynamic_chord(area_mm2, span_mm)
    chord_m = chord_mm / 1000.0
    area_m2 = area_mm2 / 1e6

    balance_frac = hinge_from_le_mm / chord_mm
    aft_chord_mm = chord_mm - hinge_from_le_mm

    q = dynamic_pressure(top_speed_kmh)
    hinge_moment_nm = ch_eff * q * area_m2 * chord_m
    required_nm = hinge_moment_nm * SAFETY_FACTOR
    required_oz_in = required_nm * 141.612

    return {
        "surface_type": "elevator",
        "chord_mm": round(chord_mm, 2),
        "mac_mm": round(mac_mm, 2),
        "balance_fraction": round(balance_frac * 100, 1),
        "aft_chord_mm": round(aft_chord_mm, 2),
        "ch_eff": ch_eff,
        "dynamic_pressure_pa": round(q, 1),
        "hinge_moment_nm": round(hinge_moment_nm, 3),
        "hinge_moment_oz_in": round(hinge_moment_nm * 141.612, 1),
        "required_torque_nm": round(required_nm, 3),
        "required_torque_oz_in": round(required_oz_in, 1),
        "safety_factor": SAFETY_FACTOR,
    }


def recommend_servos(required_oz_in: float, top_n: int = 3) -> list:
    """
    Return top_n servos that meet the torque requirement, sorted by margin.

    Uses the highest voltage torque rating available for each servo.
    """
    candidates = []
    for servo in SERVO_DB:
        max_torque = max(t["oz_in"] for t in servo["torque"])
        max_torque_entry = max(servo["torque"], key=lambda t: t["oz_in"])
        if max_torque >= required_oz_in:
            margin_pct = (max_torque - required_oz_in) / required_oz_in * 100
            candidates.append({
                "name": servo["name"],
                "category": servo["category"],
                "type": servo["type"],
                "weight_g": servo["weight_g"],
                "torque_specs": servo["torque"],
                "speed_specs": servo["speed"],
                "max_torque_oz_in": round(max_torque, 1),
                "max_torque_voltage_v": max_torque_entry["voltage_v"],
                "margin_pct": round(margin_pct, 1),
                "required_oz_in": round(required_oz_in, 1),
            })

    candidates.sort(key=lambda s: s["margin_pct"])
    return candidates[:top_n]


def _kg_or_g(kg_cm: float) -> str:
    """Return torque as kg·cm or g·cm depending on magnitude."""
    if kg_cm < 1.0:
        return f"{kg_cm * 1000:.0f} g·cm"
    return f"{kg_cm:.2f} kg·cm"


def format_torque_specs(torque_specs: list) -> str:
    parts = []
    for t in torque_specs:
        parts.append(
            f"{t['oz_in']} oz-in / {_kg_or_g(t['kg_cm'])} @ {t['voltage_v']}V"
        )
    return "  |  ".join(parts)


def format_speed_specs(speed_specs: list) -> str:
    parts = []
    for s in speed_specs:
        parts.append(f"{s['sec_60deg']}s/60° @ {s['voltage_v']}V")
    return "  |  ".join(parts)


def run_tail_elevator_selection(
    total_area_mm2: float,
    span_per_half_mm: float,
    chord_mm: float,
    hinge_from_le_mm: float,
    cruise_kmh: float,
    top_speed_kmh: float,
    surface_type: str = "stabilator",
) -> None:
    """
    Full servo selection report for a split horizontal tail.

    Parameters
    ----------
    total_area_mm2    : total area of both halves combined (mm²)
    span_per_half_mm  : span of one half (mm)
    chord_mm          : chord of the surface front to back (mm)
    hinge_from_le_mm  : hinge/pivot distance from leading edge (mm)
    cruise_kmh        : cruise airspeed (km/h)
    top_speed_kmh     : maximum airspeed (km/h)
    surface_type      : 'stabilator' or 'elevator'
    """
    area_per_half = total_area_mm2 / 2.0

    if surface_type == "stabilator":
        result = hinge_moment_stabilator(
            area_per_half, span_per_half_mm, chord_mm, hinge_from_le_mm,
            top_speed_kmh
        )
    else:
        result = hinge_moment_elevator(
            area_per_half, span_per_half_mm, chord_mm, hinge_from_le_mm,
            top_speed_kmh
        )

    recommendations = recommend_servos(result["required_torque_oz_in"])

    req_nm  = result["required_torque_nm"]
    req_oiz = result["required_torque_oz_in"]
    req_kg  = req_nm * 10.197          # N·m → kg·cm
    req_disp = (_kg_or_g(req_kg) if req_kg < 100
                else f"{req_kg:.2f} kg·cm")

    hm_nm   = result["hinge_moment_nm"]
    hm_kg   = hm_nm * 10.197
    hm_disp = (_kg_or_g(hm_kg) if hm_kg < 100 else f"{hm_kg:.2f} kg·cm")

    print("=" * 68)
    print("  TAIL SERVO SELECTION REPORT")
    print("=" * 68)
    print(f"  Surface type        : {result['surface_type'].capitalize()}")
    print(f"  Total area          : {total_area_mm2:.2f} mm²")
    print(f"  Area per half       : {area_per_half:.2f} mm²")
    print(f"  Span per half       : {span_per_half_mm:.2f} mm")
    print(f"  Chord (front→back)  : {result['chord_mm']:.2f} mm")
    print(f"  Mean aero chord     : {result['mac_mm']:.2f} mm")
    print(f"  Hinge from LE       : {hinge_from_le_mm:.1f} mm")
    print(f"  Cruise speed        : {cruise_kmh} km/h")
    print(f"  Top speed           : {top_speed_kmh} km/h")
    print("-" * 68)
    print("  AERODYNAMICS")
    print("-" * 68)
    if result["surface_type"] == "stabilator":
        print(f"  Pivot position      : {result['pivot_fraction']:.1f}% of chord")
        print(f"  Aero centre (AC)    : {result['ac_fraction']:.1f}% of chord"
              f"  ({result['ac_position_mm']:.1f} mm from LE)")
        print(f"  Moment arm (pivot→AC): {result['moment_arm_mm']:.2f} mm")
        if result["over_balanced"]:
            print("  *** WARNING: pivot is aft of AC — surface is over-balanced.")
            print("      Risk of flutter. Consider moving pivot forward.")
    else:
        print(f"  Balance ratio       : {result['balance_fraction']:.1f}% of chord")
        print(f"  Aft chord           : {result['aft_chord_mm']:.2f} mm")
        print(f"  Ch effective        : {result['ch_eff']}")
    print(f"  Dynamic pressure    : {result['dynamic_pressure_pa']:.1f} Pa")
    print(f"  Hinge moment        : {hm_nm:.3f} N·m  "
          f"({result['hinge_moment_oz_in']:.1f} oz-in / {hm_disp})")
    print(f"  Required torque     : {req_nm:.3f} N·m  "
          f"({req_oiz:.1f} oz-in / {req_disp})"
          f"  [{SAFETY_FACTOR}× safety factor]")
    print("=" * 68)
    print("  SERVO RECOMMENDATIONS  (per half, one servo each)")
    print("=" * 68)

    if not recommendations:
        print("  No servos in database meet the torque requirement.")
        print("  Consider a custom or industrial servo.")
    else:
        for i, s in enumerate(recommendations, 1):
            max_kg = s["max_torque_oz_in"] / 13.89
            need_kg = s["required_oz_in"] / 13.89
            print(f"\n  [{i}] {s['name']}  ({s['category']}, {s['type']})"
                  f"  —  {s['weight_g']} g")
            print(f"      Torque : {format_torque_specs(s['torque_specs'])}")
            print(f"      Speed  : {format_speed_specs(s['speed_specs'])}")
            print(f"      Margin : +{s['margin_pct']:.1f}% above requirement"
                  f"  (need {s['required_oz_in']:.1f} oz-in"
                  f" / {_kg_or_g(need_kg)},"
                  f" rated {s['max_torque_oz_in']:.1f} oz-in"
                  f" / {_kg_or_g(max_kg)}"
                  f" @ {s['max_torque_voltage_v']}V)")

    print("\n" + "=" * 68)


if __name__ == "__main__":
    # --- Your aircraft ---
    # 56674.31253 mm² is the area of ONE half (half of original 98503.57 mm²)
    # Pass it directly as area_per_half; total_area_mm2 = 2 × half
    run_tail_elevator_selection(
        total_area_mm2=56674.31253 * 2,   # both halves combined
        span_per_half_mm=258.13507,
        chord_mm=299.79,
        hinge_from_le_mm=63.0,
        cruise_kmh=100,
        top_speed_kmh=160,
        surface_type="stabilator",
    )
