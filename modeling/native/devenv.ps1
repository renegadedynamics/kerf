<#
.SYNOPSIS
  Dot-source to load the MSVC developer environment and the project-local CMake/Ninja.
    . native/devenv.ps1 [-Rid win-x64|win-arm64]
#>
param([ValidateSet('win-x64', 'win-arm64')] [string] $Rid = 'win-x64')

if (-not $env:VCToolsInstallDir) {
  $installer = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer"
  $env:PATH = "$installer;$env:PATH"   # VsDevCmd calls vswhere by name
  $vs = & (Join-Path $installer 'vswhere.exe') -latest -products * `
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
  if (-not $vs) { throw 'Visual Studio with C++ tools not found' }
  Import-Module (Join-Path $vs 'Common7\Tools\Microsoft.VisualStudio.DevShell.dll')
  $arch = if ($Rid -eq 'win-arm64') { 'arm64' } else { 'amd64' }
  $hostArch = if ($env:PROCESSOR_ARCHITECTURE -eq 'ARM64') { 'arm64' } else { 'amd64' }
  Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments "-arch=$arch -host_arch=$hostArch" | Out-Null
}

$tools = Join-Path (Split-Path -Parent $PSScriptRoot) '.deps\tools\Scripts'
if ((Test-Path $tools) -and -not ($env:PATH -like "*$tools*")) { $env:PATH = "$tools;$env:PATH" }
