// Configuration de la navigation entre les pages
export interface PageNav {
  title: string;
  href: string;
}

export interface NavigationItem {
  current: string;
  previous?: PageNav;
  next?: PageNav;
}

// Définition de l'ordre des pages de documentation
export const navigationMap: Record<string, NavigationItem> = {
  // Semaine 1
  "/docs/semaine-1": {
    current: "/docs/semaine-1",
    previous: { title: "Pré-sélection", href: "/pre-selection" },
    next: { title: "Électronique", href: "/docs/semaine-1/electronique" },
  },
  "/docs/semaine-1/electronique": {
    current: "/docs/semaine-1/electronique",
    previous: { title: "Semaine 1", href: "/docs/semaine-1" },
    next: { title: "Gyroscope & Accéléromètre", href: "/docs/semaine-1/electronique/gyroscope" },
  },
  "/docs/semaine-1/electronique/gyroscope": {
    current: "/docs/semaine-1/electronique/gyroscope",
    previous: { title: "Électronique", href: "/docs/semaine-1/electronique" },
    next: { title: "Communication I2C", href: "/docs/semaine-1/electronique/i2c" },
  },
  "/docs/semaine-1/electronique/i2c": {
    current: "/docs/semaine-1/electronique/i2c",
    previous: { title: "Gyroscope & Accéléromètre", href: "/docs/semaine-1/electronique/gyroscope" },
    next: { title: "Affichage LCD", href: "/docs/semaine-1/electronique/lcd" },
  },
  "/docs/semaine-1/electronique/lcd": {
    current: "/docs/semaine-1/electronique/lcd",
    previous: { title: "Communication I2C", href: "/docs/semaine-1/electronique/i2c" },
    next: { title: "IT", href: "/docs/semaine-1/it" },
  },
  "/docs/semaine-1/it": {
    current: "/docs/semaine-1/it",
    previous: { title: "Affichage LCD", href: "/docs/semaine-1/electronique/lcd" },
    next: { title: "Classe Robot", href: "/docs/semaine-1/it/robot" },
  },
  "/docs/semaine-1/it/robot": {
    current: "/docs/semaine-1/it/robot",
    previous: { title: "IT", href: "/docs/semaine-1/it" },
    next: { title: "Bras Robotique", href: "/docs/semaine-1/it/robotic-arm" },
  },
  "/docs/semaine-1/it/robotic-arm": {
    current: "/docs/semaine-1/it/robotic-arm",
    previous: { title: "Classe Robot", href: "/docs/semaine-1/it/robot" },
    next: { title: "Robot à Roues", href: "/docs/semaine-1/it/wheeled-robot" },
  },
  "/docs/semaine-1/it/wheeled-robot": {
    current: "/docs/semaine-1/it/wheeled-robot",
    previous: { title: "Bras Robotique", href: "/docs/semaine-1/it/robotic-arm" },
    next: { title: "Diagrammes UML", href: "/docs/semaine-1/it/uml" },
  },
  "/docs/semaine-1/it/uml": {
    current: "/docs/semaine-1/it/uml",
    previous: { title: "Robot à Roues", href: "/docs/semaine-1/it/wheeled-robot" },
    next: { title: "Mécanique", href: "/docs/semaine-1/mecanique" },
  },
  "/docs/semaine-1/mecanique": {
    current: "/docs/semaine-1/mecanique",
    previous: { title: "Diagrammes UML", href: "/docs/semaine-1/it/uml" },
    next: { title: "CAO", href: "/docs/semaine-1/mecanique/cao" },
  },
  "/docs/semaine-1/mecanique/cao": {
    current: "/docs/semaine-1/mecanique/cao",
    previous: { title: "Mécanique", href: "/docs/semaine-1/mecanique" },
    next: { title: "Semaine 2", href: "/docs/semaine-2" },
  },

  // Semaine 2
  "/docs/semaine-2": {
    current: "/docs/semaine-2",
    previous: { title: "CAO", href: "/docs/semaine-1/mecanique/cao" },
    next: { title: "Électronique", href: "/docs/semaine-2/electronique" },
  },
  "/docs/semaine-2/electronique": {
    current: "/docs/semaine-2/electronique",
    previous: { title: "Semaine 2", href: "/docs/semaine-2" },
    next: { title: "Hardware", href: "/docs/semaine-2/electronique/hardware" },
  },
  "/docs/semaine-2/electronique/hardware": {
    current: "/docs/semaine-2/electronique/hardware",
    previous: { title: "Électronique", href: "/docs/semaine-2/electronique" },
    next: { title: "Software & Firmware", href: "/docs/semaine-2/electronique/software-firmware" },
  },
  "/docs/semaine-2/electronique/software-firmware": {
    current: "/docs/semaine-2/electronique/software-firmware",
    previous: { title: "Hardware", href: "/docs/semaine-2/electronique/hardware" },
    next: { title: "Transmission", href: "/docs/semaine-2/electronique/transmission" },
  },
  "/docs/semaine-2/electronique/transmission": {
    current: "/docs/semaine-2/electronique/transmission",
    previous: { title: "Software & Firmware", href: "/docs/semaine-2/electronique/software-firmware" },
    next: { title: "PCB", href: "/docs/semaine-2/electronique/pcb" },
  },
  "/docs/semaine-2/electronique/pcb": {
    current: "/docs/semaine-2/electronique/pcb",
    previous: { title: "Transmission", href: "/docs/semaine-2/electronique/transmission" },
    next: { title: "Boîte Noire", href: "/docs/semaine-2/electronique/boite-noire" },
  },
  "/docs/semaine-2/electronique/boite-noire": {
    current: "/docs/semaine-2/electronique/boite-noire",
    previous: { title: "PCB", href: "/docs/semaine-2/electronique/pcb" },
    next: { title: "Résultats", href: "/docs/semaine-2/electronique/results-demonstration" },
  },
  "/docs/semaine-2/electronique/results-demonstration": {
    current: "/docs/semaine-2/electronique/results-demonstration",
    previous: { title: "Boîte Noire", href: "/docs/semaine-2/electronique/boite-noire" },
    next: { title: "Troubleshooting", href: "/docs/semaine-2/electronique/troubleshooting" },
  },
  "/docs/semaine-2/electronique/troubleshooting": {
    current: "/docs/semaine-2/electronique/troubleshooting",
    previous: { title: "Résultats", href: "/docs/semaine-2/electronique/results-demonstration" },
    next: { title: "Améliorations", href: "/docs/semaine-2/electronique/possible-improvements" },
  },
  "/docs/semaine-2/electronique/possible-improvements": {
    current: "/docs/semaine-2/electronique/possible-improvements",
    previous: { title: "Troubleshooting", href: "/docs/semaine-2/electronique/troubleshooting" },
    next: { title: "IT", href: "/docs/semaine-2/it" },
  },
  "/docs/semaine-2/it": {
    current: "/docs/semaine-2/it",
    previous: { title: "Améliorations", href: "/docs/semaine-2/electronique/possible-improvements" },
    next: { title: "Sensor Publisher", href: "/docs/semaine-2/it/sensor-publisher" },
  },
  "/docs/semaine-2/it/sensor-publisher": {
    current: "/docs/semaine-2/it/sensor-publisher",
    previous: { title: "IT", href: "/docs/semaine-2/it" },
    next: { title: "Sensor Subscriber", href: "/docs/semaine-2/it/sensor-subscriber" },
  },
  "/docs/semaine-2/it/sensor-subscriber": {
    current: "/docs/semaine-2/it/sensor-subscriber",
    previous: { title: "Sensor Publisher", href: "/docs/semaine-2/it/sensor-publisher" },
    next: { title: "ROS2 Sensor Evaluation", href: "/docs/semaine-2/it/ros2-sensor-evaluation" },
  },
  "/docs/semaine-2/it/ros2-sensor-evaluation": {
    current: "/docs/semaine-2/it/ros2-sensor-evaluation",
    previous: { title: "Sensor Subscriber", href: "/docs/semaine-2/it/sensor-subscriber" },
    next: { title: "Streamlit Dashboard", href: "/docs/semaine-2/it/streamlit-dashboard" },
  },
  "/docs/semaine-2/it/streamlit-dashboard": {
    current: "/docs/semaine-2/it/streamlit-dashboard",
    previous: { title: "ROS2 Sensor Evaluation", href: "/docs/semaine-2/it/ros2-sensor-evaluation" },
    next: { title: "Launch Configuration", href: "/docs/semaine-2/it/launch-configuration" },
  },
  "/docs/semaine-2/it/launch-configuration": {
    current: "/docs/semaine-2/it/launch-configuration",
    previous: { title: "Streamlit Dashboard", href: "/docs/semaine-2/it/streamlit-dashboard" },
    next: { title: "Mécanique", href: "/docs/semaine-2/mecanique" },
  },
  "/docs/semaine-2/mecanique": {
    current: "/docs/semaine-2/mecanique",
    previous: { title: "Launch Configuration", href: "/docs/semaine-2/it/launch-configuration" },
    next: { title: "Contraintes", href: "/docs/semaine-2/mecanique/contraintes" },
  },
  "/docs/semaine-2/mecanique/contraintes": {
    current: "/docs/semaine-2/mecanique/contraintes",
    previous: { title: "Mécanique", href: "/docs/semaine-2/mecanique" },
    next: { title: "Semaine 3", href: "/docs/semaine-3" },
  },

  // Semaine 3
  "/docs/semaine-3": {
    current: "/docs/semaine-3",
    previous: { title: "Contraintes", href: "/docs/semaine-2/mecanique/contraintes" },
    next: { title: "Électronique", href: "/docs/semaine-3/electronique" },
  },
  "/docs/semaine-3/electronique": {
    current: "/docs/semaine-3/electronique",
    previous: { title: "Semaine 3", href: "/docs/semaine-3" },
    next: { title: "Afficheur 7 Segments", href: "/docs/semaine-3/electronique/afficheur-7-servo-segments" },
  },
  "/docs/semaine-3/electronique/afficheur-7-servo-segments": {
    current: "/docs/semaine-3/electronique/afficheur-7-servo-segments",
    previous: { title: "Électronique", href: "/docs/semaine-3/electronique" },
    next: { title: "Code 7 Segments", href: "/docs/semaine-3/electronique/code-7-servo-segments" },
  },
  "/docs/semaine-3/electronique/code-7-servo-segments": {
    current: "/docs/semaine-3/electronique/code-7-servo-segments",
    previous: { title: "Afficheur 7 Segments", href: "/docs/semaine-3/electronique/afficheur-7-servo-segments" },
    next: { title: "IT", href: "/docs/semaine-3/it" },
  },
  "/docs/semaine-3/it": {
    current: "/docs/semaine-3/it",
    previous: { title: "Code 7 Segments", href: "/docs/semaine-3/electronique/code-7-servo-segments" },
    next: { title: "Introduction", href: "/docs/semaine-3/it/introduction" },
  },
  "/docs/semaine-3/it/introduction": {
    current: "/docs/semaine-3/it/introduction",
    previous: { title: "IT", href: "/docs/semaine-3/it" },
    next: { title: "Pathfinding", href: "/docs/semaine-3/it/algorithme-pathfinding" },
  },
  "/docs/semaine-3/it/algorithme-pathfinding": {
    current: "/docs/semaine-3/it/algorithme-pathfinding",
    previous: { title: "Introduction", href: "/docs/semaine-3/it/introduction" },
    next: { title: "Cartographie NAV2", href: "/docs/semaine-3/it/algorithme-cartographie-nav2" },
  },
  "/docs/semaine-3/it/algorithme-cartographie-nav2": {
    current: "/docs/semaine-3/it/algorithme-cartographie-nav2",
    previous: { title: "Pathfinding", href: "/docs/semaine-3/it/algorithme-pathfinding" },
    next: { title: "Mécanique", href: "/docs/semaine-3/mecanique" },
  },
  "/docs/semaine-3/mecanique": {
    current: "/docs/semaine-3/mecanique",
    previous: { title: "Cartographie NAV2", href: "/docs/semaine-3/it/algorithme-cartographie-nav2" },
    next: { title: "Paramètres", href: "/docs/semaine-3/mecanique/parametres" },
  },
  "/docs/semaine-3/mecanique/parametres": {
    current: "/docs/semaine-3/mecanique/parametres",
    previous: { title: "Mécanique", href: "/docs/semaine-3/mecanique" },
    next: { title: "Test Final", href: "/docs/test-final" },
  },

  // Test Final
  "/docs/test-final": {
    current: "/docs/test-final",
    previous: { title: "Paramètres", href: "/docs/semaine-3/mecanique/parametres" },
    next: { title: "Finale", href: "/finale" },
  },
};

// Fonction pour obtenir la navigation d'une page
export function getPageNavigation(currentPath: string): NavigationItem | null {
  return navigationMap[currentPath] || null;
}
