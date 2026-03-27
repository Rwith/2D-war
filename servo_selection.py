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
        "waterproof": False,
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
        "waterproof": False,
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
        "waterproof": False,
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
        "waterproof": False,
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
        "waterproof": False,
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
        "waterproof": False,
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
        "waterproof": False,
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
        "waterproof": False,
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
        "waterproof": False,
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
        "waterproof": False,
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
        "waterproof": False,
    },
    # --- Waterproof 9g class ---
    {
        "name": "Hitec HS-5086WP",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 27.1, "kg_cm": 1.95},
            {"voltage_v": 6.0, "oz_in": 34.7, "kg_cm": 2.5},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.14},
            {"voltage_v": 6.0, "sec_60deg": 0.12},
        ],
        "weight_g": 19.9,
        "type": "digital",
        "category": "9g",
        "waterproof": True,
    },
    # --- Waterproof 17g class ---
    {
        "name": "Savox SW-0250MG",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 22.2, "kg_cm": 1.6},
            {"voltage_v": 6.0, "oz_in": 27.8, "kg_cm": 2.0},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.13},
            {"voltage_v": 6.0, "sec_60deg": 0.12},
        ],
        "weight_g": 13.4,
        "type": "digital",
        "category": "17g",
        "waterproof": True,
    },
    # --- Waterproof standard class ---
    {
        "name": "Savox SW-0231MG",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 55.6, "kg_cm": 4.0},
            {"voltage_v": 6.0, "oz_in": 69.4, "kg_cm": 5.0},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.17},
            {"voltage_v": 6.0, "sec_60deg": 0.14},
        ],
        "weight_g": 56.0,
        "type": "digital",
        "category": "standard",
        "waterproof": True,
    },
    # --- Waterproof large class ---
    {
        "name": "Savox SW-1210SG",
        "torque": [
            {"voltage_v": 6.0, "oz_in": 361.1, "kg_cm": 26.0},
            {"voltage_v": 7.4, "oz_in": 430.6, "kg_cm": 31.0},
        ],
        "speed": [
            {"voltage_v": 6.0, "sec_60deg": 0.15},
            {"voltage_v": 7.4, "sec_60deg": 0.13},
        ],
        "weight_g": 96.0,
        "type": "digital",
        "category": "large",
        "waterproof": True,
    },
    {
        "name": "Savox SW-2290SG",
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
        "waterproof": True,
    },
    # --- 9 g class ---
    {
        "name": "Emax ES08MD II",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 30.6, "kg_cm": 2.2},
            {"voltage_v": 6.0, "oz_in": 34.7, "kg_cm": 2.5},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.09},
            {"voltage_v": 6.0, "sec_60deg": 0.07},
        ],
        "weight_g": 8.5,
        "type": "digital",
        "category": "9g",
        "waterproof": False,
    },
    {
        "name": "KST DS113MG",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 20.8, "kg_cm": 1.5},
            {"voltage_v": 6.0, "oz_in": 30.5, "kg_cm": 2.2},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.09},
            {"voltage_v": 6.0, "sec_60deg": 0.07},
        ],
        "weight_g": 8.0,
        "type": "digital",
        "category": "9g",
        "waterproof": False,
    },
    # --- 17 g class ---
    {
        "name": "Hitec HS-85MG+",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 41.7, "kg_cm": 3.0},
            {"voltage_v": 6.0, "oz_in": 48.6, "kg_cm": 3.5},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.15},
            {"voltage_v": 6.0, "sec_60deg": 0.14},
        ],
        "weight_g": 16.6,
        "type": "digital",
        "category": "17g",
        "waterproof": False,
    },
    {
        "name": "Blue Bird BMS-306BB",
        "torque": [
            {"voltage_v": 4.8, "oz_in": 47.2, "kg_cm": 3.4},
            {"voltage_v": 6.0, "oz_in": 57.1, "kg_cm": 4.1},
        ],
        "speed": [
            {"voltage_v": 4.8, "sec_60deg": 0.13},
            {"voltage_v": 6.0, "sec_60deg": 0.11},
        ],
        "weight_g": 17.5,
        "type": "digital",
        "category": "17g",
        "waterproof": False,
    },
]

AIR_DENSITY_KG_M3 = 1.225  # sea level, ISA
SAFETY_FACTOR     = 2.0
GUST_FACTOR       = 1.35   # storm/turbulence gust load multiplier (~35% extra)
AC_POSITION       = 0.25   # aerodynamic centre as fraction of chord (thin airfoil)


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


def recommend_servos(
    required_oz_in: float,
    top_n: int = 3,
    waterproof_only: bool = False,
) -> list:
    """
    Return top_n servos that meet the torque requirement, sorted by margin.

    Uses the highest voltage torque rating available for each servo.

    Parameters
    ----------
    required_oz_in  : minimum torque the servo must provide (oz-in)
    top_n           : number of results to return
    waterproof_only : if True, only return servos rated waterproof
    """
    candidates = []
    for servo in SERVO_DB:
        if waterproof_only and not servo.get("waterproof", False):
            continue
        max_torque = max(t["oz_in"] for t in servo["torque"])
        max_torque_entry = max(servo["torque"], key=lambda t: t["oz_in"])
        if max_torque >= required_oz_in:
            margin_pct = (max_torque - required_oz_in) / required_oz_in * 100
            candidates.append({
                "name": servo["name"],
                "category": servo["category"],
                "type": servo["type"],
                "weight_g": servo["weight_g"],
                "waterproof": servo.get("waterproof", False),
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
    waterproof: bool = False,
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

    gust = GUST_FACTOR if waterproof else 1.0
    gust_required_oz_in = result["required_torque_oz_in"] * gust
    recommendations = recommend_servos(gust_required_oz_in, waterproof_only=waterproof)

    req_nm   = result["required_torque_nm"] * gust
    req_oiz  = gust_required_oz_in
    req_kg   = req_nm * 10.197
    req_disp = (_kg_or_g(req_kg) if req_kg < 100 else f"{req_kg:.2f} kg·cm")

    hm_nm   = result["hinge_moment_nm"]
    hm_kg   = hm_nm * 10.197
    hm_disp = (_kg_or_g(hm_kg) if hm_kg < 100 else f"{hm_kg:.2f} kg·cm")

    wp_tag = "  [WATERPROOF + GUST RATED]" if waterproof else ""

    print("=" * 68)
    print(f"  TAIL SERVO SELECTION REPORT{wp_tag}")
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
    if waterproof:
        print(f"  Gust factor         : {GUST_FACTOR}×  (storm/turbulence loading)")
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
    sf_note = f"{SAFETY_FACTOR}× SF × {GUST_FACTOR}× gust" if waterproof else f"{SAFETY_FACTOR}× safety factor"
    print(f"  Required torque     : {req_nm:.3f} N·m  "
          f"({req_oiz:.1f} oz-in / {req_disp})"
          f"  [{sf_note}]")
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
            wp_mark = " ✓WP" if s["waterproof"] else ""
            print(f"\n  [{i}] {s['name']}{wp_mark}  ({s['category']}, {s['type']})"
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


def run_aileron_selection(
    label: str,
    area_per_side_mm2: float,
    span_mm: float,
    chord_mm: float,
    hinge_from_le_mm: float,
    top_speed_kmh: float,
    waterproof: bool = False,
) -> None:
    """
    Servo selection report for a single aileron panel (one side).

    Parameters
    ----------
    label             : descriptive name, e.g. 'Outer aileron'
    area_per_side_mm2 : planform area of one aileron panel (mm²)
    span_mm           : span of the panel (mm)
    chord_mm          : aileron chord front to back (mm)
    hinge_from_le_mm  : hinge distance from leading edge of aileron (mm)
    top_speed_kmh     : maximum airspeed (km/h)
    """
    # Derive ch_eff from balance ratio — plain flap approximation
    balance_frac = hinge_from_le_mm / chord_mm
    ch_eff = round(0.09 * (1.0 - 2.0 * balance_frac), 4)

    result = hinge_moment_elevator(
        area_per_side_mm2, span_mm, chord_mm, hinge_from_le_mm,
        top_speed_kmh, ch_eff=ch_eff,
    )
    recommendations = recommend_servos(result["required_torque_oz_in"])

    gust    = GUST_FACTOR if waterproof else 1.0
    req_nm  = result["required_torque_nm"] * gust
    req_oiz = result["required_torque_oz_in"] * gust
    req_kg  = req_nm * 10.197
    req_disp = _kg_or_g(req_kg)

    recommendations = recommend_servos(req_oiz, waterproof_only=waterproof)

    hm_nm  = result["hinge_moment_nm"]
    hm_kg  = hm_nm * 10.197
    hm_disp = _kg_or_g(hm_kg)

    wp_tag  = "  [WATERPROOF + GUST RATED]" if waterproof else ""
    sf_note = f"{SAFETY_FACTOR}× SF × {GUST_FACTOR}× gust" if waterproof else f"{SAFETY_FACTOR}× safety factor"

    print("=" * 68)
    print(f"  AILERON SERVO SELECTION — {label.upper()}{wp_tag}")
    print("=" * 68)
    print(f"  Area (per side)     : {area_per_side_mm2:.2f} mm²")
    print(f"  Span                : {span_mm:.2f} mm")
    print(f"  Chord               : {chord_mm:.2f} mm")
    print(f"  Hinge from LE       : {hinge_from_le_mm:.5f} mm"
          f"  ({balance_frac * 100:.1f}% balance)")
    print(f"  Ch effective        : {ch_eff}")
    print(f"  Top speed           : {top_speed_kmh} km/h")
    if waterproof:
        print(f"  Gust factor         : {GUST_FACTOR}×  (storm/turbulence loading)")
    print("-" * 68)
    print(f"  Dynamic pressure    : {result['dynamic_pressure_pa']:.1f} Pa")
    print(f"  Hinge moment        : {hm_nm:.4f} N·m"
          f"  ({result['hinge_moment_oz_in']:.2f} oz-in / {hm_disp})")
    print(f"  Required torque     : {req_nm:.4f} N·m"
          f"  ({req_oiz:.2f} oz-in / {req_disp})"
          f"  [{sf_note}]")
    print("=" * 68)
    print("  SERVO RECOMMENDATIONS  (one per panel)")
    print("=" * 68)

    if not recommendations:
        print("  No servos in database meet the torque requirement.")
    else:
        for i, s in enumerate(recommendations, 1):
            max_kg = s["max_torque_oz_in"] / 13.89
            need_kg = s["required_oz_in"] / 13.89
            wp_mark = " ✓WP" if s["waterproof"] else ""
            print(f"\n  [{i}] {s['name']}{wp_mark}  ({s['category']}, {s['type']})"
                  f"  —  {s['weight_g']} g")
            print(f"      Torque : {format_torque_specs(s['torque_specs'])}")
            print(f"      Speed  : {format_speed_specs(s['speed_specs'])}")
            print(f"      Margin : +{s['margin_pct']:.1f}% above requirement"
                  f"  (need {req_oiz:.2f} oz-in / {_kg_or_g(need_kg)},"
                  f" rated {s['max_torque_oz_in']:.1f} oz-in"
                  f" / {_kg_or_g(max_kg)} @ {s['max_torque_voltage_v']}V)")

    print("\n" + "=" * 68 + "\n")


if __name__ == "__main__":
    # --- Horizontal stabilator ---
    run_tail_elevator_selection(
        total_area_mm2=56674.31253 * 2,
        span_per_half_mm=258.13507,
        chord_mm=299.79,
        hinge_from_le_mm=63.0,
        cruise_kmh=100,
        top_speed_kmh=160,
        surface_type="stabilator",
        waterproof=True,
    )

    # --- Ailerons ---
    AILERON_CHORD_MM  = 47.93980
    AILERON_HINGE_MM  = 1.54395   # hinge rod centre from LE

    run_aileron_selection(
        label="Outer aileron (high speed)",
        area_per_side_mm2=5297.05266,
        span_mm=121.21270,
        chord_mm=AILERON_CHORD_MM,
        hinge_from_le_mm=AILERON_HINGE_MM,
        top_speed_kmh=160,
        waterproof=True,
    )

    run_aileron_selection(
        label="Inner aileron (low-medium speed)",
        area_per_side_mm2=7944.91055,
        span_mm=181.92023,
        chord_mm=AILERON_CHORD_MM,
        hinge_from_le_mm=AILERON_HINGE_MM,
        top_speed_kmh=100,
        waterproof=True,
    )
