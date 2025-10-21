export const sidebarItems = [
  {
    title: "Pré-sélection",
    href: "/pre-selection",
    iconName: "CheckCircle2",
    items: [
      {
        title: "Semaine 1 - Débutants",
        href: "/docs/semaine-1",
        items: [
          {
            title: "Électronique",
            href: "/docs/semaine-1/electronique",
            items: [
              { title: "Gyroscope & Accéléromètre", href: "/docs/semaine-1/electronique/gyroscope" },
              { title: "Circuit I2C", href: "/docs/semaine-1/electronique/i2c" },
              { title: "Affichage LCD", href: "/docs/semaine-1/electronique/lcd" },
            ]
          },
          {
            title: "IT",
            href: "/docs/semaine-1/it",
            items: [
              { title: "Classe Robot", href: "/docs/semaine-1/it/robot" },
              { title: "Bras Robotique", href: "/docs/semaine-1/it/robotic-arm" },
              { title: "Robot à Roues", href: "/docs/semaine-1/it/wheeled-robot" },
              { title: "Diagrammes UML", href: "/docs/semaine-1/it/uml" },
            ]
          },
          {
            title: "Mécanique",
            href: "/docs/semaine-1/mecanique",
            items: [
              { title: "Mécanique Débutant", href: "/docs/semaine-1/mecanique/cao" },
            ]
          }
        ]
      },
      {
        title: "Semaine 2 - Intermédiaires",
        href: "/docs/semaine-2",
        items: [
          {
            title: "Électronique",
            href: "/docs/semaine-2/electronique",
            items: [
              { title: "Boîte Noire & Station de Contrôle", href: "/docs/semaine-2/electronique/boite-noire" },
              { title: "Hardware", href: "/docs/semaine-2/electronique/hardware" },
              { title: "Software & Firmware", href: "/docs/semaine-2/electronique/software-firmware" },
              { title: "Résultats & Démonstration", href: "/docs/semaine-2/electronique/results-demonstration" },
              { title: "Améliorations Possibles", href: "/docs/semaine-2/electronique/possible-improvements" },
              { title: "Dépannage", href: "/docs/semaine-2/electronique/troubleshooting" }
            ]
          },
          {
            title: "IT",
            href: "/docs/semaine-2/it",
            items: [
              { title: "ROS2 Sensor Evaluation", href: "/docs/semaine-2/it/ros2-sensor-evaluation" },
              { title: "Launch Configuration", href: "/docs/semaine-2/it/launch-configuration" },
              { title: "Sensor Publisher", href: "/docs/semaine-2/it/sensor-publisher" },
              { title: "Sensor Subscriber", href: "/docs/semaine-2/it/sensor-subscriber" },
              { title: "Streamlit Dashboard", href: "/docs/semaine-2/it/streamlit-dashboard" },
            ]
          },
          {
            title: "Mécanique",
            href: "/docs/semaine-2/mecanique",
            items: [
              { title: "Mécanique Intermédiaire", href: "/docs/semaine-2/mecanique/contraintes" },
            ]
          }
        ]
      },
      {
        title: "Semaine 3 - Avancés",
        href: "/docs/semaine-3",
        items: [
          {
            title: "Électronique",
            href: "/docs/semaine-3/electronique",
            items: [
              { title: "Matériel Afficheur 7 Segments", href: "/docs/semaine-3/electronique/afficheur-7-servo-segments" },
              { title: "Code Afficheur 7 Segments", href: "/docs/semaine-3/electronique/code-7-servo-segments" },
            ]
          },
          {
            title: "IT",
            href: "/docs/semaine-3/it",
            items: [
              { title: "Pathfinding", href: "/docs/semaine-3/it/algorithme-pathfinding" },
              { title: "Cartographie Nav2", href: "/docs/semaine-3/it/algorithme-cartographie-nav2" },
            ]
          },
          {
            title: "Mécanique",
            href: "/docs/semaine-3/mecanique",
            items: [
              { title: "Mécanique Avancé", href: "/docs/semaine-3/mecanique/parametres" },
            ]
          }
        ]
      },
      {
        title: "Test Final",
        href: "/docs/test-final",
        items: [
          {
            title: "Électronique",
            href: "/docs/test-final/electronique",
            items: [
              { title: "Rapport Électronique", href: "/docs/test-final/electronique/rapport-electronique" },
              { title: "Arduino Code", href: "/docs/test-final/electronique/arduino-code" },
            ]
          },
          {
            title: "IT",
            href: "/docs/test-final/it",
            items: [
              { title: "Classification Couleur", href: "/docs/test-final/it/classification-couleur" },
              { title: "Dashboard Industriel", href: "/docs/test-final/it/dashboard-industriel" },
              { title: "Interface Monitoring ROS", href: "/docs/test-final/it/interface-monitoring-ros" },
            ]
          },
          {
            title: "Mécanique",
            href: "/docs/test-final/mecanique",
            items: [
              { title: "Documentation Finale", href: "/docs/test-final/mecanique/documentation-finale" },
            ]
          }
        ]
      },
    ],
  },
  {
    title: "Finale",
    href: "/finale",
    iconName: "Target",
    items: [
      { title: "Présentation finale", href: "/finale" },
      { title: "Projets sélectionnés", href: "/finale#projets" },
    ],
  },
]
