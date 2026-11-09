namespace Renegade.BladeBumper;

/// <summary>
/// All dimensions in mm, for a shaft of SHAFT_D where the body clamps, its diameter growing TAPER_D mm per metre
/// towards the handle. (JP) = his spec, (meas) = measured, the rest are my choices. The design was laid out round a
/// 35 mm shaft; on another, everything round the shaft grows with it: each surface moves out along its normal by the
/// growth in radius, DR, so every clearance to the shaft stays, things that sit on a surface stay put along it, and a
/// corner between two surfaces follows that corner. SHAFT_D and TAPER_D are the only fields, so a size is its value.
/// </summary>
public sealed record Params(double SHAFT_D = 35.0, double TAPER_D = 3.0, double FACET_DEG = 60.0, double U_SHIFT = 0.0,
                            double? LOW_SCREW_Z = null, double TOP_SCREW_OUT = 0.0)
{
    // --- shaft and rubber
    // SHAFT_D: where the body clamps, just inboard of the sleeve (JP). TAPER_D: diameter growth, mm per metre towards the handle
    public double TAPER => TAPER_D / 2000;   // radius growth per mm
    public double R => SHAFT_D / 2;
    public double DR => R - 17.5;         // growth in radius over the 35 mm shaft the design was laid out on
    // FACET_DEG: the facets' normals, from +Z. U_SHIFT: the pockets, sleds and the cavity's lower side, moved this far up
    // the facet. LOW_SCREW_Z: null, beside the clamp's insert (v0.15); a height, above it, as far in as it allows.
    // TOP_SCREW_OUT: the top screws moved out, so the O-ring's loop round them clears the pockets.
    public double S => Math.Sin(FACET_DEG * Math.PI / 180);
    public double C => Math.Cos(FACET_DEG * Math.PI / 180);
    public double T_CORNER => Math.Tan(FACET_DEG / 2 * Math.PI / 180);   // a corner between the top and a facet moves out DR T_CORNER sideways, DR up
    public double SLEEVE_R => R + 3.5;    // context only
    public double SLEEVE_GAP => 6.0;      // door bumper to sleeve: the door backs off past the LDC, then lifts; a setting
    public double RUBBER_T => 1.0;        // 37 mm bore on a 35 mm shaft (JP)
    public double R_BORE => R + RUBBER_T;
    public double RUBBER_L => 50.0;       // (JP)
    public double RUBBER_FRONT => 2.0;    // past the body's front, inside the bumper's front wall; the rest goes towards the door

    // --- body and clamp
    public double BODY_L => 30.0;         // (JP)
    // the clamp: a cap under the shaft, pulled up into the body by four bolts from below (JP)
    public double SPLIT_Z => 3.0;         // body's flat underside, below the axis (JP: within 2-3 mm of the centreline)
    public double SPLIT_GAP => 0.5;       // body to cap; fully closed it squeezes 1 mm rubber by 25 % top and bottom
    public double CLAMP_BOTTOM => R + 4.5;   // cap's underside, below the axis
    public double CLAMP_HALF => 16.0 + 0.4 * DR;   // half-width of the cap's underside: the sides taper in to it; it only has to hold the bore
    public double CLAMP_BOTTOM_R => 5.0;  // its corners
    public double CLAMP_Y => R_BORE + 3.5;        // bolts, each side: between the bore and the lower door screws
    public double BOLT_PITCH => 15.0;     // along the shaft, a pair each side
    public double M3_INSERT_HOLE => 4.5;  // M3 x 4 heat-set inserts in the body's underside
    public double CLAMP_SEAT_Z => -6.5;   // bolt heads' seat in the cap, in recesses opening through its tapered side
    public double CLAMP_BOLT_L => 8.0;
    public double CLAMP_HOLE_L => CLAMP_BOTTOM + 8.0;   // from below the cap, past the split
    public double M3_CBORE_D => 6.2;

    // --- arm
    public double ARM_D => 10.0;
    public double ARM_ID => 8.0;   // carbon tube (meas)
    public double ARM_GAP => 4.0;         // arm to the shaft, at the least (JP); the sockets' gap to it sets it (SOCKET_SHAFT_GAP)
    public double ARM_OUT => 100.0;       // from the body's front (JP)
    public double RIM_T => 2.0;           // (JP): arm goes 28 into the body
    public double SOCKET_D => 10.3;       // bonded with epoxy
    public double RIM_HOLE => 8.0;

    // --- the sealed arch (cross-section), shared by the body's back face and the door
    public double N_FLOOR => R_BORE + 3.7;        // the boards' floor: 3.7 clear of the bore for the O-ring and its lands
    public double N_ROOF => N_FLOOR + 8.0;         // door's roof: 8.0 headroom, for a sled under the shuttle boards and a USB-C plug
    public double U_LOW => -6.9 + U_SHIFT;          // door cavity's lower side
    public double Z_TOP_FLOOR => N_FLOOR;    // the cavity's floor over the shaft, under the arm hole
    public double Z_ROOF => N_ROOF + 2.8;         // door's roof on top: the top screws' bosses fit under it
    public double WALL => 1.7;            // door wall (4 perimeters)
    public double FLANGE_RAISE => 3.0;    // the bare flanges stand this far proud of the body and door: flush with the bumpers (JP)
    public double BODY_FLANGE_L => 6.0;
    public double DOOR_FLANGE_L => 7.0;   // the door's holds the screws' insert bosses

    // --- body pockets
    public double POCKET_N1 => N_FLOOR + 6.5;      // 6.5 deep
    public (double Lo, double Hi) POCKET_U => (-6.6 + U_SHIFT, 14.4 + U_SHIFT);   // 5.6 clear of the top screws
    public double POCKET_CHAMFER => 1.0;  // off each pocket's upper outer corner, where the O-ring rounds the top screw
    public double FRONT_WALL => 2.0;      // pocket's end to the body's front: pockets are 28 long

    // --- door and seal
    public double DOOR_BORE_R => R_BORE + 0.7;    // 0.7 clear of the rubber
    public double DOOR_END_GAP => 0.5;    // last board to the door's end wall
    public double ORING_CS => 1.5;        // cord, in a groove in the body's back face
    public double GROOVE_W => 2.1;
    public double GROOVE_DEPTH => 1.1;
    public double SEAL_LAND => 0.8;
    public double SEAL_ROUND => 1.5;      // the sealed face's corners are rounded to this, then the groove follows it
    public double SEAL_ROUND_NEAR => 9.0 + TOP_SCREW_OUT; // ... its convex corners only within this of the top screws
    // door screws: through the body from the arm side, into heat-set inserts in the door
    public double M3_HOLE => 3.4;
    public double M3_INSERT_D => 4.0;
    public double DOOR_INSERT_DEPTH => 6.0;
    public double DOOR_BOSS_R => 3.2;     // round the insert, inside the door
    public double M3_HEAD_D => 5.5;
    public double M3_HEAD_H => 3.0;
    public double M3_L => 35.0;
    public (double Y, double Z) TOP_SCREW => (13.5 + DR * T_CORNER + TOP_SCREW_OUT, 31.0 + DR);   // each side: the top corners
    public (double Y, double Z) LOW_SCREW => LOW_SCREW_Z is double z ? LowScrewAt(z) : (26.7 + DR, 0.3);   // each side: below the pockets

    /// <summary>
    /// A lower door screw above the clamp's insert, as far in as it allows: 0.85 of plastic to the insert's hole below
    /// (or beside) it; 1.5 round the door's insert to its bore, and 0.5 round the body bumper's hole for its head to that
    /// bumper's bore.
    /// </summary>
    (double Y, double Z) LowScrewAt(double z)
    {
        var need = 0.85 + M3_HOLE / 2 + M3_INSERT_D / 2;
        var dz = Math.Max(0.0, z - (-SPLIT_Z + M3_INSERT_HOLE));
        var r = Math.Max(DOOR_BORE_R + M3_INSERT_D / 2 + 1.5, R_BORE + 0.5 + 3.5 + 0.5);
        return (Math.Max(CLAMP_Y + Math.Sqrt(Math.Max(0.0, need * need - dz * dz)), Math.Sqrt(r * r - z * z)), z);
    }
    public double SCREW_CLEAR => 5.6;     // screw axis to the sealed face: insert, land, groove, land
    public double SEAT_PAD => 1.0;        // plastic round the caps' bolt heads at their seats
    public double HULL_PAD => 2.0;        // the outline keeps this round each screw: 1.7 of wall outside a 3.4 hole
    public double BUMPER_HOLE_R => 3.5;   // in the body bumper's front wall, round each head

    // --- bumpers
    public double BUMPER_T => 3.0;        // (JP)
    public double BUMPER_CLR => 0.05;
    public double BUMPER_SKIRT => 2.5;    // the body bumper reaches this far below the split, over the clamp (JP: 2-3)
    public double BORE_CHAMFER => 0.6;    // the straight bore's bottom edges, so they don't catch the rubber
    public double BUMPER_DOOR_BORE_R => R_BORE + 0.9;
    public double BUMPER_BORE_R => R_BORE + 0.5;  // through its front wall

    // --- boot and its sockets
    public double AIR_TOP => 3.0;
    public double AIR_SIDE => 2.0;
    public double AIR_BOTTOM => 1.5;   // round the arm (JP: 0.7 more underneath)
    public double BOOT_WALL => 1.4;
    public double SOCKET_L => 10.0;
    public double SOCKET_CLR => 0.1;      // glue gap, each side
    public double SOCKET_WALL => 1.4;
    public double BOOT_SHAFT_GAP => 1.0;  // the boot's underside follows the bare shaft, this far off it
    public double SOCKET_SHAFT_GAP => 1.0;   // (JP: at least 1) the sockets' undersides, at the housing end where the shaft is
                                             // fattest: this sets the arm's height
    public double BOOT_CHAMFER => 1.0;    // where their vertical sides meet that curve (JP)
    public double BOOT_END_FLOOR => 0.8;  // the boot's ends sit fully in the sockets: thin only between the arm and the shaft
    public double SOCKET_FLOOR => 0.8;
    public double BOOT_END_R => 3.9;      // the boot ends' (and the sockets' holes') corners: a convex rounded rectangle
    public double RIB_H => 0.4;
    public double GROOVE_H => 0.5;
    public double RIB_L => 1.5;
    public double[] RIB_AT => [2.0, 6.0];   // rib starts, from the socket's mouth end
    public double RIB_LOW_Z => R + 4.0;      // top and sides only: no room under the arm

    // --- coil (cut-down Seeed board first; the coil PCB's plane is the same)
    public double COIL_D => 16.0;         // the coil itself (JP)
    public double COIL_HEAD_D => 18.0;    // (JP)
    public double COIL_TAIL => 19.0;      // from the coil's centre (JP)
    public double COIL_TAIL_W => 7.6;     // at least this (JP); this one fits the tube
    public double COIL_T => 1.6;
    public double COIL_PAST_TIP => 14.0;  // coil centre past the tube's end, as the coil PCB
    public double SPIGOT_L => 10.0;
    public double SPIGOT_D => 7.8;
    public double COLLAR_L => 1.5;
    public double SLOT_W => 7.7;
    public double SLOT_T => 1.7;

    // --- housing
    public double HOUSING_L => 30.0;      // (JP), centred on the coil
    public double COIL_AIR => 1.2;
    public double COIL_WALL => 0.8;   // coil -> air -> TPU skin -> plate; skin is two perimeters
    public double CHAMBER_Y => 11.5;
    public double CHAMBER_ABOVE => 3.5;   // coil's top to the chamber's roof: the gap opens by up to ~2 under drive
    public double CHAMBER_END => 2.5;     // coil head to the chamber's end
    public double HOUSING_ROOF => 2.2;
    public (double Y, double Z, double R) HOUSING_TOP => (13.0 + DR * T_CORNER, 30.0 + DR, 3.0);   // the body's section, trimmed to this top corner
    public double HOUSING_SPLIT_Z => 3.0;
    public double HOUSING_GAP => 0.5;
    public double HOUSING_BOTTOM => CLAMP_BOTTOM;
    public double HOUSING_BOLT_Y => R + 5.0;      // nut pockets 1.5 clear of the shaft
    public double HOUSING_SEAT_Z => -7.0;
    public double M3_NUT_AF => 5.5;
    public double M3_NUT_H => 4.0;
    public double HOUSING_NUT_Z => 0.0;        // nylocs dropped down shafts from the top onto a 3 mm TPU ledge over the split (JP)
    public double NUT_SHAFT_AF => 5.7;
    public double PLUG_AF => 5.5;              // hexagonal TPU plugs glued into the shafts afterwards, flush with the outside (JP)
    public double PLUG_GAP => 1.5;             // above the nut, clear of the bolt's tip
    public double HOUSING_BOLT_L => 12.0;
    public double HOUSING_RAMP => 12.0;        // the housing's outside is the convex hull of the boot's socket and itself over this
    public double HOUSING_END_C => 3.0;        // 45 deg chamfer on the housing's (and its cap's) far end, so it doesn't catch on the ground (JP)
    public double STADIUM_END => 132.3;   // the boot's inner section runs into the housing to here

    // --- plate and saddle
    public double PLATE_L => 20.0;
    public double PLATE_T => 1.0;   // 20 x 20: the coil PCB's plan; stops 4 mm past the tube's end
    public double SADDLE_W => 28.0;       // across the shaft: the housing bears on it beside the chamber
    public double SADDLE_END => 2.0;      // beyond the plate at the far end
    public double SADDLE_LOW_Z => Math.Sqrt(R * R - SADDLE_W / 2 * (SADDLE_W / 2)) - 1.5;   // its edges hang this far below the shaft's top
    public double SEAT_CLR => 0.05;       // saddle and housing to the shaft

    // --- boards, as the earlier designs
    public double XIAO_L => 21.2;
    public double XIAO_W => 17.9;
    public double XIAO_T => 1.4;
    public double XIAO_TOP => 2.0;   // (meas)
    public double USB_H => 3.2;
    public double USB_W => 9.0;
    public double USB_OUT => 2.2;
    public double USB_IN => 7.3;
    public double SHUTTLE_L => 22.1;
    public double SHUTTLE_W => 14.05;
    public double SHUTTLE_T => 1.6;        // (meas)
    public double SHUTTLE_PINS => 2.5;
    public double SHUTTLE_CHIP => 1.0;   // pins trimmed to stubs (JP)
    public double SHUTTLE_ROW => 17.78;
    public double FLASH_L => 20.3;
    public double FLASH_W => 17.7;
    public double FLASH_H => 3.5;
    public double REG_L => 18.0;
    public double REG_W => 8.0;
    public double REG_H => 5.6;    // 1.8 V stripboard, parts laid flat
    public double LDC_L => 32.0;
    public double LDC_W => 20.0;
    public double LDC_H => 2.8;   // coil half cut off (meas)
    public double BAT_L => 30.5;
    public double BAT_W => 20.5;
    public double BAT_H => 5.3;   // 502030 with swell allowance (est.)
    public double ANT_L => 35.0;
    public double ANT_W => 8.5;
    public double ANT_T => 0.25;   // FPC antenna (meas)
    public double TAPE => 0.5;            // foam tape under the LDC in its pocket
    public double SHUTTLE_LIFT => 1.2;    // pads under the shuttles' ends, clear of the chip underneath
    public double BAT_LIFT => 0.4;
    public double BOND => 0.1;            // epoxy under the boards on their sleds

    // --- sleds: one per side, slid in through the O-ring's opening, snapped in; bond them in later (JP)
    public double SLED_CLR => 0.2;        // under the sled, to the door's floor
    public double SLED_T => 1.0;
    public (double Lo, double Hi) SLED_U => (-6.6 + U_SHIFT, 13.6 + U_SHIFT);   // across the facet: 20.2 wide
    public double SLED_RAIL_W => 1.0;
    public double SLED_RAIL_H => 0.8;   // side rails: they fence the boards and run under the door's lips
    public double SLED_STOP_W => 1.0;     // fences across the sled, between and at the ends of the boards
    public double SLED_PAD_L => 2.0;      // the shuttles' pads, at their ends
    public double SLED_SIDE_CLR => 0.3;   // the sled's sides to the door's wall and rib
    public double LIP_OVER => 0.5;
    public double LIP_GAP => 0.2;
    public double LIP_T => 0.8;   // the door's lips over the rails
    public double RIB_W => 1.3;           // the door's rib along each sled's upper side, carrying a lip
    public double DETENT_BUMP => 0.5;     // on the rib, into a notch in the rail: 0.2 of it is the click
    public double DETENT_L => 1.5;
    public double REG_X => -10.0;         // the regulator lies flat on the floor over the shaft, from here back
    public double USB_TIP_X => -4.5;      // just past the LDC's end

    // --- the boot's print aid: printed with it, on end, then cut away (JP: the boot is tall and unstable on end)
    public double TOWER_GAP => 0.4;       // wall to the boot's thick middle: it can't sway further than this
    public double TOWER_WALL => 1.2;      // three perimeters
    public double TOWER_ZGAP => 0.3;      // support gap: the shelf under the ledge where the middle overhangs the bottom socket end
    public double TOWER_COLLAR => 15.0;   // how far the wall reaches up the thick middle past the ledge (JP: the base is what needs holding)
}
