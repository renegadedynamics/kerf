# The blade bumper's enclosure

`Bumper.cs`, `Components.cs` and `P.cs` are the Kerf model of the blade sensor's bumper enclosure, by Renegade
Dynamics, from their hardware repository (renegadedynamics/hardware, `products/rowing-strain/enclosure/kerf/BladeBumper/`,
commit `3ae3778`, branch `feature/kerf-0.4.0`: "Kerf model: Kerf 0.4.0, the housing's end a collapse chamfer"). It is
included in Kerf with the owner's permission (decision 2008), as a user's part for the release check and the rig. The
three files are as they were committed, unchanged; the model's design is described in that repository.

Only `Rebuild.cs` is Kerf's: the model keeps each part in a static `Lazy<T>`, and a type of that name in the model's
namespace takes its place, so that each timed run builds the whole model again (`Rebuild.Forget`).

The rest of that folder (exports, renders, drawings, and the spec tests against its build123d model) isn't here. The
case reads every printed part, the print aid and the placed components; `../Expected.cs` holds each one's volume, validity
and face count.
