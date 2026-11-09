# The blade bumper's enclosure, v0.16

`Bumper.cs`, `Components.cs` and `Params.cs` are the Kerf model of the blade sensor's bumper enclosure, by Renegade
Dynamics, from their hardware repository (renegadedynamics/hardware, `products/rowing-strain/enclosure/kerf/BladeBumper/`,
commit `9ab5fb0`, branch `feature/facet-angle`: "Blade bumper v0.16: facets at 47.5 deg, lower door screws above the clamp
inserts"). It is included in Kerf with the owner's permission (decision 2008), as a user's part for the release check and
the rig, beside `../enclosure`, the same model at v0.11 (13C). The three files are as they were committed, unchanged; the
model's design is described in that repository.

Since v0.11 the model has been laid out for three shaft sizes (35, 50 and 58 mm), with its dimensions in a record
(`Params.cs`) and each cached value built once per size. The case builds the 35 mm size, which is what the script builds
by default (`dotnet run`); the script's tests build all three.

Only `Rebuild.cs` is Kerf's: the model keeps each value in a `Lazy<T>`, and a type of that name in the model's namespace
takes its place, so that each timed run builds the whole model again (`Rebuild.Forget`). The model is a library of its
own (`Kerf.UserCases.EnclosureV016.csproj`), as its types have the v0.11 model's names; `Kerf.UserCases` refers to it by
the alias `v016`.

The rest of that folder (exports, renders, drawings, and the spec tests against its build123d model) isn't here. The case
reads every printed part, the print aid and the placed components, as the v0.11 case does; `../Expected.cs` holds each
one's volume, validity and face count.
