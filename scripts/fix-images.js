const fs = require('fs');
const path = require('path');

// Fonction pour remplacer les chemins d'images dans un fichier
function fixImagesInFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  
  // Remplace les chemins d'images
  const fixedContent = content
    .replace(/src="\/Documentation\//g, 'src="/2025-Team-IFRI-Docs/Documentation/')
    .replace(/src="\/placeholder-/g, 'src="/2025-Team-IFRI-Docs/placeholder-')
    .replace(/src: "\/Documentation\//g, 'src: "/2025-Team-IFRI-Docs/Documentation/')
    .replace(/src: "\/placeholder-/g, 'src: "/2025-Team-IFRI-Docs/placeholder-')
    .replace(/image: "\/placeholder-/g, 'image: "/2025-Team-IFRI-Docs/placeholder-');
  
  if (content !== fixedContent) {
    fs.writeFileSync(filePath, fixedContent);
    console.log(`✅ Fixed: ${filePath}`);
  }
}

// Parcourt tous les fichiers .tsx et .ts
function fixAllImages(dir) {
  const files = fs.readdirSync(dir);
  
  files.forEach(file => {
    const filePath = path.join(dir, file);
    const stat = fs.statSync(filePath);
    
    if (stat.isDirectory() && !file.startsWith('.') && file !== 'node_modules') {
      fixAllImages(filePath);
    } else if (file.endsWith('.tsx') || file.endsWith('.ts')) {
      fixImagesInFile(filePath);
    }
  });
}

console.log('🔧 Fixing image paths for GitHub Pages...');
fixAllImages('./components');
fixAllImages('./app');
console.log('✅ All image paths fixed!');
