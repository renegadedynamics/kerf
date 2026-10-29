# Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
# Publishes the WebAssembly demo, serves it, runs it in a headless Chrome (or Edge) with ?auto=1 over a folder of .brep
# files, waits for its report, then compares the buffers with win-x64's (the benchmark project's wasm-compare mode).
#   pwsh samples/OcctNet.WasmDemo/run-headless.ps1 -Data <folder of .brep files> [-Out <folder>] [-Root]
#       [-Properties OcctSourceDir=...,Build123dNetRoot=...]   (MSBuild properties, as for the tests)
param(
    [Parameter(Mandatory = $true)][string]$Data,
    [string]$Out = (Join-Path ([IO.Path]::GetTempPath()) "occtnet-wasmdemo"),
    [switch]$Root,                      # root the OcctNet assemblies (no trimming of them), as Bento.Web does its own
    [int]$Port = 8642,
    [int]$TimeoutMinutes = 60,
    # data/occ's terrain.brep needs 5.6 GB of managed heap to mesh (on win-x64 too): more than 32-bit WebAssembly has
    [string]$Exclude = 'terrain.brep',
    [string[]]$Properties = @()
)
$ErrorActionPreference = 'Stop'
$MSBuildArgs = @($Properties | ForEach-Object { "-p:$_" })
$repo = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$publish = Join-Path $Out 'publish'
$report = Join-Path $Out 'report.json'
Remove-Item $report -ErrorAction SilentlyContinue

$extra = @()
if ($Root) { $extra += '-p:RootOcctNet=true' }
dotnet publish (Join-Path $repo 'samples\OcctNet.WasmDemo') -c Release -o $publish @extra @MSBuildArgs
if ($LASTEXITCODE -ne 0) { throw 'publish failed' }

$browser = @("$env:ProgramFiles\Google\Chrome\Application\chrome.exe",
             "${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe") | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $browser) { throw 'no Chrome or Edge found' }

$server = Start-Process node -ArgumentList @((Join-Path $PSScriptRoot 'serve.mjs'), '-root', (Join-Path $publish 'wwwroot'), '-data', $Data,
                                              '-port', $Port, '-out', $report, '-exit', '-exclude', $Exclude) -PassThru -NoNewWindow
$browserProfile = Join-Path $Out 'browser-profile'
$chrome = Start-Process $browser -ArgumentList @('--headless=new', "--user-data-dir=$browserProfile", '--use-angle=swiftshader',
                                                 '--enable-unsafe-swiftshader', '--no-first-run', '--window-size=1200,900',
                                                 "http://localhost:$Port/?auto=1") -PassThru
try {
    $deadline = (Get-Date).AddMinutes($TimeoutMinutes)
    while (-not (Test-Path $report) -and (Get-Date) -lt $deadline -and -not $server.HasExited) { Start-Sleep -Seconds 2 }
}
finally {
    Get-Process -Id $chrome.Id -ErrorAction SilentlyContinue | Stop-Process -Force
    Get-CimInstance Win32_Process -Filter "Name like '%chrome%' or Name like '%msedge%'" |
        Where-Object { $_.CommandLine -like "*$browserProfile*" } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    if (-not $server.HasExited) { Stop-Process -Id $server.Id -Force }
}
if (-not (Test-Path $report)) { throw "no report within $TimeoutMinutes minutes" }

dotnet run -c Release --project (Join-Path $repo 'benchmarks\OcctNet.Benchmarks') @MSBuildArgs -- wasm-compare $report $Data
