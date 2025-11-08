# Script pour copier les assets vers le dossier public
$sourceDir = "Documentation\semaine-1\electronique\images"
$destDir = "public\Documentation\semaine-1\electronique\images"

# Créer le dossier de destination s'il n'existe pas
if (!(Test-Path $destDir)) {
    New-Item -ItemType Directory -Path $destDir -Force
}

# Copier tous les fichiers
Get-ChildItem $sourceDir | ForEach-Object {
    Copy-Item $_.FullName "$destDir\$($_.Name)" -Force
    Write-Host "Copié: $($_.Name)"
}

Write-Host "Copie terminée !"
