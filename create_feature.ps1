# Temporary script to create the new feature
param(
    [Parameter(Mandatory=$true)]
    [string]$FeatureDescription,
    
    [Parameter(Mandatory=$true)]
    [int]$Number,
    
    [Parameter(Mandatory=$true)]
    [string]$ShortName
)

# Execute the main script with the parameters
& "$PSScriptRoot\.specify\scripts\powershell\create-new-feature.ps1" -FeatureDescription $FeatureDescription -Number $Number -ShortName $ShortName