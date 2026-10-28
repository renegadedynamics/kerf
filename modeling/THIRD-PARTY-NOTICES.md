# Third-party notices

build123d.NET is a port of build123d, and its packages ship native libraries built from the projects
below. The full licence texts are in [licenses/](licenses), which the packages carry too.

## build123d

- **What:** the Python library this is a port of. The .NET code follows its design, algorithms and
  messages.
- **Source:** https://github.com/gumyr/build123d
- **Licence:** Apache License 2.0, copyright the build123d contributors (its NOTICE is in [NOTICE](NOTICE)).
  See [licenses/build123d-Apache-2.0.txt](licenses/build123d-Apache-2.0.txt).

## Open CASCADE Technology (OCCT) 8.0.1

- **What:** the geometry kernel, shipped as the `TK*.dll` libraries in `Build123d.Occt`.
- **Source:** https://github.com/Open-Cascade-SAS/OCCT, tag `V8_0_1`, built unmodified by
  `native/occt`.
- **Licence:** GNU Lesser General Public License 2.1 with the Open CASCADE exception, copyright Open
  CASCADE SAS. See [licenses/OCCT-LGPL-2.1.txt](licenses/OCCT-LGPL-2.1.txt) and
  [licenses/OCCT-LGPL-exception.txt](licenses/OCCT-LGPL-exception.txt).

This software makes use of, and is based on, facilities provided by the Open CASCADE Technology
software. OCCT is linked dynamically: the libraries can be replaced by other builds of the same
version, and its source is available from the link above.

## FreeType 2.14.3

- **What:** font rendering, used by OCCT's TKService. It is shipped as `freetype.dll`.
- **Source:** https://github.com/freetype/freetype, tag `VER-2-14-3`.
- **Licence:** the FreeType Project License (FTL). Portions of this software are copyright © The
  FreeType Project (https://freetype.org). All rights reserved. See
  [licenses/FreeType-FTL.txt](licenses/FreeType-FTL.txt).

## Bento.Collections

- **What:** the immutable collections under `ShapeList`. It is a NuGet dependency and isn't shipped
  in these packages.
- **Licence:** MIT.
