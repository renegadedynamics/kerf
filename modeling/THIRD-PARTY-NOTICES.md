# Third-party notices

Kerf.Modeling is derived from build123d (the build123d contributors), ported to C# and redesigned for .NET. It isn't
build123d and isn't endorsed by its authors. Its package ships no native libraries: it runs on Kerf.Kernel, a package
dependency under its own licence. The full licence texts are in [licenses/](licenses), which the package carries too.

## build123d

- **What:** the Python library this is derived from. The .NET code follows its design and algorithms; its API is
  redesigned for .NET.
- **Source:** https://github.com/gumyr/build123d
- **Licence:** Apache License 2.0, copyright the build123d contributors (its NOTICE is in [NOTICE](NOTICE)).
  See [licenses/build123d-Apache-2.0.txt](licenses/build123d-Apache-2.0.txt).

## Kerf.Kernel (derived from Open CASCADE Technology 8.0.1)

- **What:** the geometry kernel, the `Kerf.Kernel.*` packages this package depends on (the repository's `kernel`
  folder). It isn't shipped in this package.
- **Licence:** GNU Lesser General Public License 2.1 with the Open CASCADE exception, as OCCT's, copyright Open CASCADE
  SAS and the Kerf contributors. See [licenses/OCCT-LGPL-2.1.txt](licenses/OCCT-LGPL-2.1.txt) and
  [licenses/OCCT-LGPL-exception.txt](licenses/OCCT-LGPL-exception.txt).

Until unit 10B the package could also be built on native OCCT and FreeType, whose libraries it then shipped. That
backend is gone, and so is FreeType's licence from this package; the 0.1.0 previews that shipped FreeType carry it.

