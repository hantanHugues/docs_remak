"use client";

import { Navbar } from "@/components/navbar"
import { DocsSidebar } from "@/components/docs-sidebar"
import { AnimatedSection } from "@/components/animated-section"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Code, Network, Cpu, ArrowRight, ArrowLeft, BookOpen, Target, Clock } from "lucide-react"
import Link from "next/link"

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

const sections = [
  {
    title: "ROS2 Sensor Evaluation",
    description: "Vue d'ensemble du projet et architecture générale ROS2",
    href: "/docs/semaine-2/it/ros2-sensor-evaluation",
    icon: Code,
    difficulty: "Intermédiaire",
    duration: "2h",
    color: "from-purple-500 to-pink-500"
  },
  {
    title: "Launch Configuration",
    description: "Configuration et gestion des fichiers launch pour automation",
    href: "/docs/semaine-2/it/launch-configuration",
    icon: Network,
    difficulty: "Intermédiaire",
    duration: "1h",
    color: "from-blue-500 to-cyan-500"
  },
  {
    title: "Sensor Publisher",
    description: "Node ROS2 pour génération et publication de données capteurs",
    href: "/docs/semaine-2/it/sensor-publisher",
    icon: Cpu,
    difficulty: "Intermédiaire",
    duration: "1.5h",
    color: "from-green-500 to-emerald-500"
  },
  {
    title: "Sensor Subscriber",
    description: "Reception, validation et sauvegarde des données capteurs",
    href: "/docs/semaine-2/it/sensor-subscriber",
    icon: Network,
    difficulty: "Intermédiaire", 
    duration: "1.5h",
    color: "from-orange-500 to-red-500"
  },
  {
    title: "Streamlit Dashboard",
    description: "Interface web temps réel pour visualisation et monitoring",
    href: "/docs/semaine-2/it/streamlit-dashboard",
    icon: Cpu,
    difficulty: "Intermédiaire",
    duration: "2h",
    color: "from-purple-500 to-indigo-500"
  }
]

export default function ITS2Page() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebar items={sidebarItems} />

        <main className="flex-1 min-w-0">
          {/* Hero Section */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-20 overflow-hidden bg-gradient-to-br from-purple-50 via-white to-pink-50 dark:from-purple-950/20 dark:via-background dark:to-pink-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto text-center space-y-6">
                  <Badge className="bg-purple-100 text-purple-700 border-purple-200 dark:bg-purple-900/50 dark:text-purple-300 dark:border-purple-800">
                    <Code className="w-4 h-4 mr-2" />
                    Semaine 2 - IT
                  </Badge>
                  
                  <h1 className="text-4xl md:text-6xl font-bold text-foreground">
                    IT <span className="text-purple-600">Intermédiaire</span>
                  </h1>
                  
                  <p className="text-xl text-muted-foreground max-w-2xl mx-auto leading-relaxed">
                    Plongez dans ROS2 et les architectures distribuées. Maîtrisez les communications 
                    inter-processus et la gestion avancée de capteurs.
                  </p>

                  <div className="flex flex-wrap items-center justify-center gap-4 pt-4">
                    <div className="flex items-center gap-2 text-sm text-muted-foreground">
                      <Clock className="w-4 h-4" />
                      <span>~8h de contenu</span>
                    </div>
                    <div className="flex items-center gap-2 text-sm text-muted-foreground">
                      <BookOpen className="w-4 h-4" />
                      <span>5 sections spécialisées</span>
                    </div>
                    <div className="flex items-center gap-2 text-sm text-muted-foreground">
                      <Target className="w-4 h-4" />
                      <span>Niveau intermédiaire</span>
                    </div>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Sections Grid */}
          <AnimatedSection animation="fade-in" delay={100}>
            <section className="py-16">
              <div className="container mx-auto px-4">
                <div className="max-w-6xl mx-auto">
                  <div className="text-center mb-12">
                    <h2 className="text-3xl font-bold mb-4">Sections de cours</h2>
                    <p className="text-muted-foreground max-w-2xl mx-auto">
                      Explorez ROS2 et les architectures logicielles avancées pour la robotique
                    </p>
                  </div>

                  <div className="grid md:grid-cols-1 lg:grid-cols-2 xl:grid-cols-3 gap-6">
                    {sections.map((section, index) => {
                      const Icon = section.icon
                      return (
                        <AnimatedSection key={section.title} animation="fade-up" delay={index * 100}>
                          <Card className="group hover:shadow-xl transition-all duration-300 border-2 hover:border-purple-200 dark:hover:border-purple-800">
                            <CardHeader className="pb-4">
                              <div className={`w-12 h-12 rounded-xl bg-gradient-to-r ${section.color} flex items-center justify-center mb-4 group-hover:scale-110 transition-transform duration-300`}>
                                <Icon className="w-6 h-6 text-white" />
                              </div>
                              <CardTitle className="text-xl group-hover:text-purple-600 transition-colors">
                                {section.title}
                              </CardTitle>
                              <CardDescription className="text-sm">
                                {section.description}
                              </CardDescription>
                            </CardHeader>
                            <CardContent className="pt-0">
                              <div className="flex items-center justify-between mb-4">
                                <Badge variant="secondary" className="text-xs">
                                  {section.difficulty}
                                </Badge>
                                <span className="text-xs text-muted-foreground flex items-center gap-1">
                                  <Clock className="w-3 h-3" />
                                  {section.duration}
                                </span>
                              </div>
                              <Link href={section.href}>
                                <Button className="w-full group-hover:bg-purple-600 group-hover:border-purple-600 transition-colors">
                                  Commencer
                                  <ArrowRight className="ml-2 h-4 w-4 group-hover:translate-x-1 transition-transform" />
                                </Button>
                              </Link>
                            </CardContent>
                          </Card>
                        </AnimatedSection>
                      )
                    })}
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Objectifs Section */}
          <AnimatedSection animation="fade-in" delay={200}>
            <section className="py-16 bg-muted/30">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="text-center mb-12">
                    <h2 className="text-3xl font-bold mb-4">Objectifs d'apprentissage</h2>
                    <p className="text-muted-foreground">
                      À la fin de cette section, vous maîtriserez :
                    </p>
                  </div>

                  <div className="grid md:grid-cols-2 gap-6">
                    <div className="space-y-4">
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-purple-100 dark:bg-purple-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Code className="w-3 h-3 text-purple-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Écosystème ROS2</h3>
                          <p className="text-sm text-muted-foreground">
                            Installation, configuration et utilisation des outils ROS2
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-blue-100 dark:bg-blue-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Network className="w-3 h-3 text-blue-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Architectures distribuées</h3>
                          <p className="text-sm text-muted-foreground">
                            Conception de systèmes robotiques modulaires et scalables
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="space-y-4">
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-green-100 dark:bg-green-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Cpu className="w-3 h-3 text-green-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Fusion de capteurs</h3>
                          <p className="text-sm text-muted-foreground">
                            Traitement et synchronisation de données multi-capteurs
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-orange-100 dark:bg-orange-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Target className="w-3 h-3 text-orange-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Temps réel</h3>
                          <p className="text-sm text-muted-foreground">
                            Gestion des contraintes temporelles en robotique
                          </p>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-4xl mx-auto">
              <AnimatedSection animation="fade-up">
                <div className="flex justify-between items-center pt-8 border-t">
                  <Link href="/docs/semaine-2/electronique">
                    <Button variant="outline" className="gap-2">
                      <ArrowLeft className="w-4 h-4" />
                      Électronique Semaine 2
                    </Button>
                  </Link>
                  <Link href="/docs/semaine-3">
                    <Button className="gap-2">
                      Semaine 3
                      <ArrowRight className="w-4 h-4" />
                    </Button>
                  </Link>
                </div>
              </AnimatedSection>
            </div>
          </div>
        </main>
      </div>
    </div>
  )
}
