<#
.SYNOPSIS
  Downloads OCCT's test data (the files the Draw corpus restores) into .deps/occt-testdata, and checks it.
.DESCRIPTION
  OCCT V8_0_1's own CI uses the dataset published with V7_9_0_beta1 (opencascade-dataset-7.9.0); occt-version.json
  records its URL and SHA-256. The archive is about 100 MB. The Draw-lite corpus tests find the data through the
  CSF_TestDataPath convention: .deps/occt-testdata is passed to them as OcctTestDataDir.
#>
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$version = Get-Content (Join-Path $root 'occt-version.json') -Raw | ConvertFrom-Json
$data = $version.testData
$target = Join-Path $root '.deps/occt-testdata'
$archive = Join-Path $root ".deps/$($data.file)"

if (Test-Path (Join-Path $target '.complete')) {
    "OCCT test data is in $target"
    return
}

if (-not (Test-Path $archive)) {
    Invoke-WebRequest -Uri $data.url -OutFile $archive
}

$hash = (Get-FileHash $archive -Algorithm SHA256).Hash.ToLowerInvariant()
if ($hash -ne $data.sha256) {
    throw "$archive has SHA-256 $hash, not $($data.sha256). Delete it and run this again."
}

# The archive is data, not code: extracted into a directory of its own and never executed
New-Item -ItemType Directory -Force $target | Out-Null
tar -xJf $archive -C $target
if ($LASTEXITCODE) { throw "extracting $archive failed ($LASTEXITCODE)" }
Set-Content (Join-Path $target '.complete') $hash
"OCCT test data ($($data.file)) is in $target"
