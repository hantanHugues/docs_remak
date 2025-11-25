/**
 * Script pour corriger automatiquement les boutons de navigation dans toutes les pages
 * Usage: node scripts/fix-navigation.js
 */

const fs = require('fs');
const path = require('path');

// Pattern à rechercher et remplacer
const OLD_NAVIGATION_PATTERN = /\{\/\* Navigation \*\/\}[\s\S]*?<div className="border-b bg-background\/95 backdrop-blur supports-\[backdrop-filter\]:bg-background\/60 sticky top-16 md:top-20 z-30">[\s\S]*?<\/div>[\s\S]*?<\/div>[\s\S]*?<\/div>/;

const NEW_NAVIGATION = `{/* Navigation */}
          <PageNavigation />`;

// Pattern pour l'import
const IMPORT_PATTERN = /^(import.*?from.*?["']@\/components\/navbar["'];?\n)/m;
const NEW_IMPORT_LINE = `import { PageNavigation } from "@/components/page-navigation";\n`;

// Fonction pour parcourir récursivement les dossiers
function walkDir(dir, callback) {
  fs.readdirSync(dir).forEach(f => {
    const dirPath = path.join(dir, f);
    const isDirectory = fs.statSync(dirPath).isDirectory();
    if (isDirectory) {
      walkDir(dirPath, callback);
    } else {
      callback(path.join(dir, f));
    }
  });
}

// Fonction pour vérifier si le fichier contient déjà PageNavigation
function hasPageNavigation(content) {
  return content.includes('PageNavigation') || content.includes('<PageNavigation');
}

// Fonction pour corriger un fichier
function fixFile(filePath) {
  if (!filePath.endsWith('.tsx')) return;
  
  let content = fs.readFileSync(filePath, 'utf8');
  
  // Skip si déjà corrigé
  if (hasPageNavigation(content)) {
    console.log(`✓ Déjà corrigé: ${filePath}`);
    return;
  }
  
  // Vérifier si le fichier a une ancienne navigation
  const hasOldNav = content.includes('Page Précédente') || content.includes('Page Suivante') ||
                    (content.includes('ArrowLeft') && content.includes('ArrowRight') && content.includes('sticky top-16'));
  
  if (!hasOldNav) {
    return;
  }
  
  let modified = false;
  
  // Ajouter l'import si nécessaire
  if (!content.includes('PageNavigation')) {
    const importMatch = content.match(IMPORT_PATTERN);
    if (importMatch) {
      content = content.replace(IMPORT_PATTERN, `$1${NEW_IMPORT_LINE}`);
      modified = true;
    }
  }
  
  // Remplacer l'ancienne navigation
  if (OLD_NAVIGATION_PATTERN.test(content)) {
    content = content.replace(OLD_NAVIGATION_PATTERN, NEW_NAVIGATION);
    modified = true;
  }
  
  if (modified) {
    fs.writeFileSync(filePath, content, 'utf8');
    console.log(`✅ Corrigé: ${filePath}`);
  }
}

// Lancer le script
const docsDir = path.join(__dirname, '..', 'app', 'docs');

console.log('🔍 Recherche des fichiers à corriger...\n');

try {
  walkDir(docsDir, fixFile);
  console.log('\n✨ Migration terminée !');
} catch (error) {
  console.error('❌ Erreur:', error.message);
  process.exit(1);
}
