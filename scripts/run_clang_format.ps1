#!/usr/bin/env pwsh

param (
	[Parameter(Mandatory = $true, HelpMessage = 'Path to directory with source files')]
	[ValidateNotNullOrEmpty()]
	[string]$SourcePath,

	[Parameter(HelpMessage = 'Apply fixes if applicable')]
	[switch]$Fix = $false
)

. $PSScriptRoot/_common.ps1

$SourceFiles = Get-ChildItem -Recurse -Path $SourcePath -Include ('*.cpp', '*.h')

$Outputs = [Collections.Concurrent.ConcurrentBag[psobject]]::new()

$SourceFiles | ForEach-Object -Parallel {
	$Outputs = $using:Outputs
	$Fix = $using:Fix
	$Output = $null
	$($Output = clang-format $($Fix ? '-i' : '-n') --Werror $_ *>&1) || $Outputs.Add($Output)
} -ThrottleLimit ([Environment]::ProcessorCount)

$Outputs | Where-Object { $_ -ne $null } | ForEach-Object {
	Write-Output $_
	Write-Output ''
}

exit $Outputs.IsEmpty ? 0 : 1