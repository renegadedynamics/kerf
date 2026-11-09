# 13G for Bento: one exception type from the objects

Kerf's decision 2013 (unit 13G). Nothing in Bento has to change.

- An object that can't be made from its values (`new Triangle(sideA: 1, sideB: 1, sideC: 3)`, a `Spline` through points
  1e-7 apart) now throws `Kerf.Modeling.ModelingException`, where it threw `ArgumentException`,
  `InvalidOperationException` or the kernel's own exceptions. Bento's model functions catch every exception and show
  its message (`ModelFunctions.cs`, `CellValue.FromError`), so cells show the same text as before; a kernel failure that
  escaped unwrapped now says what was being made ("slot failed: …").
- If a cell should show the error's code, `ModelingException.Error.Code` has it (`modeling.no-solution`,
  `modeling.limit-not-reached`, `modeling.object-failed`, or the kernel's), as a failed `Result`'s `Error` does.
- Each object has a `Make…` operation returning a `Result` (`Operations.MakeTriangle`, `MakeSpline`, …, with the
  constructor's parameters), which a cell could use instead of catching.
- Arguments wrong on their own (a null, too few points, a missing angle) are still `ArgumentException`s.
