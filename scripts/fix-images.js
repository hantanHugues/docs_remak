const fs = require('fs');
const path = require('path');

// Fonction pour remplacer les chemins d'images dans un fichier
function fixImagesInFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  
  // Remplace les chemins d'images et logos
  const fixedContent = content
    // Documentation images
    .replace(/src="\/Documentation\//g, 'src="/2025-Team-IFRI-Docs/Documentation/')
    .replace(/src: "\/Documentation\//g, 'src: "/2025-Team-IFRI-Docs/Documentation/')
    // Placeholder images
    .replace(/src="\/placeholder-/g, 'src="/2025-Team-IFRI-Docs/placeholder-')
    .replace(/src: "\/placeholder-/g, 'src: "/2025-Team-IFRI-Docs/placeholder-')
    .replace(/image: "\/placeholder-/g, 'image: "/2025-Team-IFRI-Docs/placeholder-')
    // Logos IFRI
    .replace(/src="\/Ifri_logo\.png/g, 'src="/2025-Team-IFRI-Docs/Ifri_logo.png')
    .replace(/src: "\/Ifri_logo\.png/g, 'src: "/2025-Team-IFRI-Docs/Ifri_logo.png')
    .replace(/src="\/IFRI-300x300\.png/g, 'src="/2025-Team-IFRI-Docs/IFRI-300x300.png')
    .replace(/src: "\/IFRI-300x300\.png/g, 'src: "/2025-Team-IFRI-Docs/IFRI-300x300.png')
    // Autres logos téléchargés
    .replace(/src="\/téléchargé\.png/g, 'src="/2025-Team-IFRI-Docs/téléchargé.png')
    .replace(/src: "\/téléchargé\.png/g, 'src: "/2025-Team-IFRI-Docs/téléchargé.png')
    .replace(/src="\/téléchargé2\.png/g, 'src="/2025-Team-IFRI-Docs/téléchargé2.png')
    .replace(/src: "\/téléchargé2\.png/g, 'src: "/2025-Team-IFRI-Docs/téléchargé2.png')
    // Pattern générique pour toutes les images .png, .jpg, .svg à la racine de public
    .replace(/src="\/([a-zA-Z0-9_-]+\.(png|jpg|jpeg|svg|gif))"/g, 'src="/2025-Team-IFRI-Docs/$1"')
    .replace(/src: "\/([a-zA-Z0-9_-]+\.(png|jpg|jpeg|svg|gif))"/g, 'src: "/2025-Team-IFRI-Docs/$1"');
  
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
