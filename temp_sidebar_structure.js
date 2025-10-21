const sidebarItems = [
  {
    title: "Pré-sélection",
    href: "/pre-selection",
    iconName: "CheckCircle2",
    items: [
      {
        title: "Semaine 1 - Débutants",
        href: "/pre-selection#semaine-1",
        items: [
          {
            title: "Électronique",
            href: "/docs/semaine-1/electronique",
            items: [
              { title: "Présentation", href: "/docs/semaine-1/electronique" },
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
              { title: "Documentation CAO", href: "/docs/semaine-1/mecanique/cao" },
              { title: "Pièces d'assemblage", href: "/docs/semaine-1/mecanique/pieces" },
              { title: "Pince mécanique", href: "/docs/semaine-1/mecanique/pince" },
              { title: "Calculs de masse", href: "/docs/semaine-1/mecanique/masse" },
            ]
          }
        ]
      },
      {
        title: "Semaine 2 - Intermédiaires",
        href: "/pre-selection#semaine-2",
        items: [
          {
            title: "Électronique",
            href: "/docs/semaine-2/electronique",
            items: [
              { title: "Boîte noire", href: "/docs/semaine-2/electronique/boite-noire" },
              { title: "PCB ATmega328P", href: "/docs/semaine-2/electronique/pcb" },
              { title: "Transmission I2C", href: "/docs/semaine-2/electronique/transmission" },
            ]
          },
          {
            title: "IT",
            href: "/docs/semaine-2/it",
            items: [
              { title: "Introduction ROS2", href: "/docs/semaine-2/it/ros2" },
              { title: "Nodes Publisher/Subscriber", href: "/docs/semaine-2/it/nodes" },
              { title: "Gestion capteurs", href: "/docs/semaine-2/it/capteurs" },
            ]
          },
          {
            title: "Mécanique",
            href: "/docs/semaine-2/mecanique",
            items: [
              { title: "Conception avancée", href: "/docs/semaine-2/mecanique/conception" },
              { title: "Maillons de chaîne", href: "/docs/semaine-2/mecanique/chaine" },
              { title: "Contraintes complexes", href: "/docs/semaine-2/mecanique/contraintes" },
            ]
          }
        ]
      },
      {
        title: "Semaine 3 - Avancés",
        href: "/pre-selection#semaine-3",
        items: [
          {
            title: "Électronique",
            href: "/docs/semaine-3/electronique",
            items: [
              { title: "Afficheur 7 segments", href: "/docs/semaine-3/electronique/7segments" },
              { title: "Servomoteurs", href: "/docs/semaine-3/electronique/servo" },
              { title: "Batteries lithium", href: "/docs/semaine-3/electronique/batterie" },
            ]
          },
          {
            title: "IT",
            href: "/docs/semaine-3/it",
            items: [
              { title: "Algorithmes Pathfinding", href: "/docs/semaine-3/it/pathfinding" },
              { title: "A* & Dijkstra", href: "/docs/semaine-3/it/algorithmes" },
              { title: "ROS2 & Gazebo", href: "/docs/semaine-3/it/gazebo" },
              { title: "Visualisation RViz2", href: "/docs/semaine-3/it/rviz" },
            ]
          },
          {
            title: "Mécanique",
            href: "/docs/semaine-3/mecanique",
            items: [
              { title: "Paramètres variables", href: "/docs/semaine-3/mecanique/parametres" },
              { title: "Impression 3D", href: "/docs/semaine-3/mecanique/3d" },
              { title: "Calculs précis", href: "/docs/semaine-3/mecanique/calculs" },
            ]
          }
        ]
      },
      { title: "Test Final", href: "/pre-selection#test-final" },
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
