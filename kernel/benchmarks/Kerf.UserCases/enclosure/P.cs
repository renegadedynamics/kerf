namespace Renegade.BladeBumper;

/// <summary>All dimensions in mm. (JP) = his spec, (meas) = measured, the rest are my choices.</summary>
public static class P
{
    // --- shaft and rubber
    public const double SHAFT_D = 35.0;        // just inboard of the sleeve (JP)
    public const double TAPER = 0.0015;        // radius growth per mm towards the handle: ~37 at the handle end (JP)
    public const double SLEEVE_GAP = 6.0;      // door bumper to sleeve: the door backs off past the LDC, then lifts; a setting
    public const double RUBBER_T = 1.0;        // 37 mm bore on a 35 mm shaft (JP)
    public const double RUBBER_L = 50.0;       // (JP)
    public const double RUBBER_FRONT = 2.0;    // past the body's front, inside the bumper's front wall; the rest goes towards the door

    // --- body and clamp
    public const double BODY_L = 30.0;         // (JP)
    // the clamp: a cap under the shaft, pulled up into the body by four bolts from below (JP)
    public const double SPLIT_Z = 3.0;         // body's flat underside, below the axis (JP: within 2-3 mm of the centreline)
    public const double SPLIT_GAP = 0.5;       // body to cap; fully closed it squeezes 1 mm rubber by 25 % top and bottom
    public const double CLAMP_BOTTOM = 22.0;   // cap's underside, below the axis
    public const double CLAMP_HALF = 16.0;     // half-width of the cap's underside: the sides taper in to it
    public const double CLAMP_BOTTOM_R = 5.0;  // its corners
    public const double CLAMP_Y = 22.0;        // bolts, each side: between the bore and the lower door screws
    public const double BOLT_PITCH = 15.0;     // along the shaft, a pair each side
    public const double M3_INSERT_HOLE = 4.5;  // M3 x 4 heat-set inserts in the body's underside
    public const double CLAMP_SEAT_Z = -6.5;   // bolt heads' seat in the cap, in recesses opening through its tapered side
    public const double CLAMP_BOLT_L = 8.0;
    public const double M3_CBORE_D = 6.2;

    // --- arm
    public const double ARM_D = 10.0, ARM_ID = 8.0;   // carbon tube (meas)
    public const double ARM_GAP = 4.0;         // arm to the 35 mm shaft (JP)
    public const double ARM_OUT = 100.0;       // from the body's front (JP)
    public const double RIM_T = 2.0;           // (JP): arm goes 28 into the body
    public const double SOCKET_D = 10.3;       // bonded with epoxy
    public const double RIM_HOLE = 8.0;

    // --- the sealed arch (cross-section), shared by the body's back face and the door
    public const double N_FLOOR = 22.2;        // the boards' floor: 3.7 clear of the bore for the O-ring and its lands
    public const double N_ROOF = 30.2;         // door's roof: 8.0 headroom, for a sled under the shuttle boards and a USB-C plug
    public const double U_LOW = -6.9;          // door cavity's lower side
    public const double Z_TOP_FLOOR = 22.2;    // the cavity's floor over the shaft, under the arm hole
    public const double Z_ROOF = 33.0;         // door's roof on top: the top screws' bosses fit under it
    public const double WALL = 1.7;            // door wall (4 perimeters)
    public const double FLANGE_RAISE = 3.0;    // the bare flanges stand this far proud of the body and door: flush with the bumpers (JP)
    public const double BODY_FLANGE_L = 6.0, DOOR_FLANGE_L = 7.0;   // the door's holds the screws' insert bosses

    // --- body pockets
    public const double POCKET_N1 = 28.7;      // 6.5 deep
    public static readonly (double Lo, double Hi) POCKET_U = (-6.6, 14.4);   // 5.6 clear of the top screws
    public const double POCKET_CHAMFER = 1.0;  // off each pocket's upper outer corner, where the O-ring rounds the top screw
    public const double FRONT_WALL = 2.0;      // pocket's end to the body's front: pockets are 28 long

    // --- door and seal
    public const double DOOR_BORE_R = 19.2;    // 0.7 clear of the rubber
    public const double DOOR_END_GAP = 0.5;    // last board to the door's end wall
    public const double ORING_CS = 1.5;        // cord, in a groove in the body's back face
    public const double GROOVE_W = 2.1, GROOVE_DEPTH = 1.1;
    public const double SEAL_LAND = 0.8;
    public const double SEAL_ROUND = 1.5;      // the sealed face's corners are rounded to this, then the groove follows it
    public const double SEAL_ROUND_NEAR = 9.0; // ... its convex corners only within this of the top screws
    // door screws: through the body from the arm side, into heat-set inserts in the door
    public const double M3_HOLE = 3.4, M3_INSERT_D = 4.0;
    public const double DOOR_INSERT_DEPTH = 6.0;
    public const double DOOR_BOSS_R = 3.2;     // round the insert, inside the door
    public const double M3_HEAD_D = 5.5, M3_HEAD_H = 3.0, M3_L = 35.0;
    public static readonly (double Y, double Z) TOP_SCREW = (13.5, 31.0);   // each side: the top corners
    public static readonly (double Y, double Z) LOW_SCREW = (26.7, 0.3);    // each side: below the pockets
    public const double SCREW_CLEAR = 5.6;     // screw axis to the sealed face: insert, land, groove, land
    public const double HULL_PAD = 2.0;        // the outline keeps this round each screw: 1.7 of wall outside a 3.4 hole
    public const double BUMPER_HOLE_R = 3.5;   // in the body bumper's front wall, round each head

    // --- bumpers
    public const double BUMPER_T = 3.0;        // (JP)
    public const double BUMPER_CLR = 0.05;
    public const double BUMPER_SKIRT = 2.5;    // the body bumper reaches this far below the split, over the clamp (JP: 2-3)
    public const double BORE_CHAMFER = 0.6;    // the straight bore's bottom edges, so they don't catch the rubber
    public const double BUMPER_DOOR_BORE_R = 19.4;
    public const double BUMPER_BORE_R = 19.0;  // through its front wall

    // --- boot and its sockets
    public const double AIR_TOP = 3.0, AIR_SIDE = 2.0, AIR_BOTTOM = 1.5;   // round the arm (JP: 0.7 more underneath)
    public const double BOOT_WALL = 1.4;
    public const double SOCKET_L = 10.0;
    public const double SOCKET_CLR = 0.1;      // glue gap, each side
    public const double SOCKET_WALL = 1.4;
    public const double BOOT_SHAFT_GAP = 0.8;  // the boot's and sockets' undersides follow the bare shaft, this far off it
    public const double BOOT_CHAMFER = 1.0;    // where their vertical sides meet that curve (JP)
    public const double BOOT_END_FLOOR = 0.8;  // the boot's ends sit fully in the sockets: thin only between the arm and the shaft
    public const double SOCKET_FLOOR = 0.8;
    public const double BOOT_END_R = 3.9;      // the boot ends' (and the sockets' holes') corners: a convex rounded rectangle
    public const double RIB_H = 0.4, GROOVE_H = 0.5;
    public const double RIB_L = 1.5;
    public static readonly double[] RIB_AT = [2.0, 6.0];   // rib starts, from the socket's mouth end
    public const double RIB_LOW_Z = 21.5;      // top and sides only: no room under the arm

    // --- coil (cut-down Seeed board first; the coil PCB's plane is the same)
    public const double COIL_D = 16.0;         // the coil itself (JP)
    public const double COIL_HEAD_D = 18.0;    // (JP)
    public const double COIL_TAIL = 19.0;      // from the coil's centre (JP)
    public const double COIL_TAIL_W = 7.6;     // at least this (JP); this one fits the tube
    public const double COIL_T = 1.6;
    public const double COIL_PAST_TIP = 14.0;  // coil centre past the tube's end, as the coil PCB
    public const double SPIGOT_L = 10.0;
    public const double SPIGOT_D = 7.8;
    public const double COLLAR_L = 1.5;
    public const double SLOT_W = 7.7, SLOT_T = 1.7;

    // --- housing
    public const double HOUSING_L = 30.0;      // (JP), centred on the coil
    public const double COIL_AIR = 1.2, COIL_WALL = 0.8;   // coil -> air -> TPU skin -> plate; skin is two perimeters
    public const double CHAMBER_Y = 11.5;
    public const double CHAMBER_ABOVE = 3.5;   // coil's top to the chamber's roof: the gap opens by up to ~2 under drive
    public const double CHAMBER_END = 2.5;     // coil head to the chamber's end
    public const double HOUSING_ROOF = 2.2;
    public static readonly (double Y, double Z, double R) HOUSING_TOP = (13.0, 30.0, 3.0);   // the body's section, trimmed to this top corner
    public const double HOUSING_SPLIT_Z = 3.0;
    public const double HOUSING_GAP = 0.5;
    public const double HOUSING_BOTTOM = CLAMP_BOTTOM;
    public const double HOUSING_BOLT_Y = 22.5;      // nut pockets 1.5 clear of the shaft
    public const double HOUSING_SEAT_Z = -7.0;
    public const double M3_NUT_AF = 5.5, M3_NUT_H = 4.0;
    public const double HOUSING_NUT_Z = 0.0;        // nylocs dropped down shafts from the top onto a 3 mm TPU ledge over the split (JP)
    public const double NUT_SHAFT_AF = 5.7;
    public const double PLUG_AF = 5.5;              // hexagonal TPU plugs glued into the shafts afterwards, flush with the outside (JP)
    public const double PLUG_GAP = 1.5;             // above the nut, clear of the bolt's tip
    public const double HOUSING_BOLT_L = 12.0;
    public const double HOUSING_RAMP = 12.0;        // the housing's outside is the convex hull of the boot's socket and itself over this
    public const double HOUSING_END_C = 3.0;        // 45 deg chamfer on the housing's (and its cap's) far end, so it doesn't catch on the ground (JP)
    public const double STADIUM_END = 132.3;   // the boot's inner section runs into the housing to here

    // --- plate and saddle
    public const double PLATE_L = 20.0, PLATE_T = 1.0;   // 20 x 20: the coil PCB's plan; stops 4 mm past the tube's end
    public const double SADDLE_W = 28.0;       // across the shaft: the housing bears on it beside the chamber
    public const double SADDLE_END = 2.0;      // beyond the plate at the far end
    public const double SADDLE_LOW_Z = 9.0;
    public const double SEAT_CLR = 0.05;       // saddle and housing to the shaft

    // --- boards, as the earlier designs
    public const double XIAO_L = 21.2, XIAO_W = 17.9, XIAO_T = 1.4, XIAO_TOP = 2.0;   // (meas)
    public const double USB_H = 3.2, USB_W = 9.0, USB_OUT = 2.2, USB_IN = 7.3;
    public const double SHUTTLE_L = 22.1, SHUTTLE_W = 14.05, SHUTTLE_T = 1.6;        // (meas)
    public const double SHUTTLE_PINS = 2.5, SHUTTLE_CHIP = 1.0;   // pins trimmed to stubs (JP)
    public const double SHUTTLE_ROW = 17.78;
    public const double FLASH_L = 20.3, FLASH_W = 17.7, FLASH_H = 3.5;
    public const double REG_L = 18.0, REG_W = 8.0, REG_H = 5.6;    // 1.8 V stripboard, parts laid flat
    public const double LDC_L = 32.0, LDC_W = 20.0, LDC_H = 2.8;   // coil half cut off (meas)
    public const double BAT_L = 30.5, BAT_W = 20.5, BAT_H = 5.3;   // 502030 with swell allowance (est.)
    public const double ANT_L = 35.0, ANT_W = 8.5, ANT_T = 0.25;   // FPC antenna (meas)
    public const double TAPE = 0.5;            // foam tape under the LDC in its pocket
    public const double SHUTTLE_LIFT = 1.2;    // pads under the shuttles' ends, clear of the chip underneath
    public const double BAT_LIFT = 0.4;
    public const double BOND = 0.1;            // epoxy under the boards on their sleds

    // --- sleds: one per side, slid in through the O-ring's opening, snapped in; bond them in later (JP)
    public const double SLED_CLR = 0.2;        // under the sled, to the door's floor
    public const double SLED_T = 1.0;
    public static readonly (double Lo, double Hi) SLED_U = (-6.6, 13.6);   // across the facet: 20.2 wide
    public const double SLED_RAIL_W = 1.0, SLED_RAIL_H = 0.8;   // side rails: they fence the boards and run under the door's lips
    public const double SLED_STOP_W = 1.0;     // fences across the sled, between and at the ends of the boards
    public const double SLED_PAD_L = 2.0;      // the shuttles' pads, at their ends
    public const double SLED_SIDE_CLR = 0.3;   // the sled's sides to the door's wall and rib
    public const double LIP_OVER = 0.5, LIP_GAP = 0.2, LIP_T = 0.8;   // the door's lips over the rails
    public const double RIB_W = 1.3;           // the door's rib along each sled's upper side, carrying a lip
    public const double DETENT_BUMP = 0.5;     // on the rib, into a notch in the rail: 0.2 of it is the click
    public const double DETENT_L = 1.5;
    public const double REG_X = -10.0;         // the regulator lies flat on the floor over the shaft, from here back
    public const double USB_TIP_X = -4.5;      // just past the LDC's end

    // --- the boot's print aid: printed with it, on end, then cut away (JP: the boot is tall and unstable on end)
    public const double TOWER_GAP = 0.4;       // wall to the boot's thick middle: it can't sway further than this
    public const double TOWER_WALL = 1.2;      // three perimeters
    public const double TOWER_ZGAP = 0.3;      // support gap: the shelf under the ledge where the middle overhangs the bottom socket end
    public const double TOWER_COLLAR = 15.0;   // how far the wall reaches up the thick middle past the ledge (JP: the base is what needs holding)
}
