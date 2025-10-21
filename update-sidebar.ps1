# Script PowerShell pour mettre à jour tous les fichiers page.tsx avec le nouveau composant sidebar

$files = @(
    "app\docs\semaine-1\electronique\gyroscope\page.tsx",
    "app\docs\semaine-1\electronique\i2c\page.tsx",
    "app\docs\semaine-1\electronique\lcd\page.tsx",
    "app\docs\semaine-1\electronique\page.tsx",
    "app\docs\semaine-1\it\page.tsx",
    "app\docs\semaine-1\it\uml\page.tsx",
    "app\docs\semaine-1\it\wheeled-robot\page.tsx",
    "app\docs\semaine-1\mecanique\cao\page.tsx",
    "app\docs\semaine-1\mecanique\masse\page.tsx",
    "app\docs\semaine-1\mecanique\page.tsx",
    "app\docs\semaine-1\mecanique\pieces\page.tsx",
    "app\docs\semaine-1\mecanique\pince\page.tsx",
    "app\docs\semaine-2\electronique\boite-noire\page.tsx",
    "app\docs\semaine-2\electronique\hardware\page.tsx",
    "app\docs\semaine-2\electronique\page.tsx",
    "app\docs\semaine-2\electronique\possible-improvements\page.tsx",
    "app\docs\semaine-2\electronique\results-demonstration\page.tsx",
    "app\docs\semaine-2\electronique\software-firmware\page.tsx",
    "app\docs\semaine-2\electronique\troubleshooting\page.tsx",
    "app\docs\semaine-2\it\capteurs\page.tsx",
    "app\docs\semaine-2\it\nodes\page.tsx",
    "app\docs\semaine-2\it\page.tsx",
    "app\docs\semaine-2\it\ros2\page.tsx",
    "app\docs\semaine-2\mecanique\chaine\page.tsx",
    "app\docs\semaine-2\mecanique\conception\page.tsx",
    "app\docs\semaine-2\mecanique\contraintes\page.tsx",
    "app\docs\semaine-2\mecanique\page.tsx",
    "app\docs\semaine-3\electronique\7segments\page.tsx",
    "app\docs\semaine-3\electronique\batterie\page.tsx",
    "app\docs\semaine-3\electronique\page.tsx",
    "app\docs\semaine-3\electronique\servo\page.tsx",
    "app\docs\semaine-3\it\algorithmes\page.tsx",
    "app\docs\semaine-3\it\gazebo\page.tsx",
    "app\docs\semaine-3\it\page.tsx",
    "app\docs\semaine-3\it\pathfinding\page.tsx",
    "app\docs\semaine-3\it\rviz\page.tsx",
    "app\docs\semaine-3\mecanique\3d\page.tsx",
    "app\docs\semaine-3\mecanique\calculs\page.tsx",
    "app\docs\semaine-3\mecanique\page.tsx",
    "app\docs\semaine-3\mecanique\parametres\page.tsx"
)

foreach ($file in $files) {
    $fullPath = Join-Path $PSScriptRoot $file
    
    if (Test-Path $fullPath) {
        Write-Host "Mise à jour de $file..."
        
        # Lire le contenu du fichier
        $content = Get-Content $fullPath -Raw
        
        # Remplacer l'import DocsSidebar par DocsSidebarWrapper
        $content = $content -replace 'import { DocsSidebar } from "@/components/docs-sidebar";', 'import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";'
        
        # Supprimer la configuration sidebarItems (pattern complexe)
        $content = $content -replace '(?s)// NOTE:.*?const sidebarItems = \[.*?\];', ''
        $content = $content -replace '(?s)const sidebarItems = \[.*?\];', ''
        
        # Remplacer l'utilisation de DocsSidebar
        $content = $content -replace '<DocsSidebar items={sidebarItems} />', '<DocsSidebarWrapper />'
        
        # Nettoyer les lignes vides multiples
        $content = $content -replace '\n\n\n+', "`n`n"
        
        # Écrire le contenu modifié
        Set-Content $fullPath $content -NoNewline
        
        Write-Host "✓ $file mis à jour"
    } else {
        Write-Host "⚠ Fichier non trouvé: $file"
    }
}

Write-Host "`nMise à jour terminée!"
