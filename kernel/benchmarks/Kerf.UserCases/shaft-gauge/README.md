# The shaft gauges

`Gauge.cs` is the Kerf model of Renegade Dynamics' rowing blade shaft gauges, from their hardware repository
(renegadedynamics/hardware, `products/rowing-strain/enclosure/kerf/ShaftGauge/Gauge.cs`, commit `031bd55`: "Shaft gauges
for 50 and 58 mm shafts"). It is included in Kerf with the owner's permission (decision 2145), as a user's part for the
release check and the rig, beside the blade bumper's enclosure (`../enclosure`, `../enclosure-v0.16`). The file is as it
was committed, unchanged; the model's design is described in it.

Each gauge is a flat rounded hexagon, 4 thick, with six U-shaped cutouts (a cylinder and a box cut for each), their
mouths filleted, a 0.4 chamfer round every rim of both faces (96 edges at 35 mm), and the cutouts' sizes and the gauge's
name engraved 0.6 deep in Liberation Sans Bold, one Boolean for each. It exercises what the enclosure doesn't: text, fillets
after Booleans, and a chamfer along a long chain of rims.

The case builds all three sizes, 35, 50 and 58 mm, as the script does by default (`dotnet run`), and reads each size's
blank (the hexagon cut, filleted and chamfered) and gauge (the blank engraved): six shapes. **Every one of them is a fillet
result:** the blanks are their corner and mouth fillets and the chamfer, and the gauges are cut from the blanks, so a change
to fillets moves all six. `../Expected.cs` holds each one's volume, validity and face count.

Only `ShaftGaugeModel.cs` (in the folder above) is Kerf's: the model keeps each size's blank and gauge in private static
dictionaries, which it empties before each run, so that each timed run builds every size again.

The rest of that folder (`Program.cs`, which exports STEP, STL and 3MF and renders each gauge, and the drawing it publishes
through the bumper's `Publisher.cs`) isn't here; `docs/shaft-gauge-feedback.md` measures them.

Fingerprints at Kerf's main `86bb21f86` when the case was added (and on unit 15D's branch, whose changes keep them) (the gauge's BREP text, SHA-256, its first 16 hex digits; not checked,
as a BREP moves with any change of rounding): 35 mm `2E13950418D1A727`, 50 mm `F27196E333CBBAA5`, 58 mm `74C4CE51273105D3`.
