"use client";

import { Navbar } from "@/components/navbar"
import { DocsSidebar } from "@/components/docs-sidebar"
import { AnimatedSection } from "@/components/animated-section"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Cpu, Zap, Battery, Monitor, ArrowRight, BookOpen, Target, Clock } from "lucide-react"
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
              { title: "Présentation", href: "/docs/semaine-3/electronique" },
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
    title: "Afficheur 7 servo Segments",
    description: "Maîtriser les systèmes d'affichage multiplexés avec contrôle servo",
    href: "/docs/semaine-3/electronique/afficheur-7-servo-segments",
    icon: Monitor,
    difficulty: "Avancé",
    duration: "2.5h",
    color: "from-red-500 to-pink-500"
  },
  {
    title: "Code 7 servo Segments",
    description: "Programmation avancée des afficheurs avec servomoteurs",
    href: "/docs/semaine-3/electronique/code-7-servo-segments",
    icon: Cpu,
    difficulty: "Avancé",
    duration: "3h",
    color: "from-blue-500 to-cyan-500"
  }
]

export default function ElectroniqueS3Page() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebar items={sidebarItems} />

        <main className="flex-1 min-w-0">
          {/* Hero Section */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-20 overflow-hidden bg-gradient-to-br from-red-50 via-white to-pink-50 dark:from-red-950/20 dark:via-background dark:to-pink-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto text-center space-y-6">
                  <Badge className="bg-red-100 text-red-700 border-red-200 dark:bg-red-900/50 dark:text-red-300 dark:border-red-800">
                    <Zap className="w-4 h-4 mr-2" />
                    Semaine 3 - Électronique
                  </Badge>
                  
                  <h1 className="text-4xl md:text-6xl font-bold text-foreground">
                    Électronique <span className="text-red-600">Avancé</span>
                  </h1>
                  
                  <p className="text-xl text-muted-foreground max-w-2xl mx-auto leading-relaxed">
                    Maîtrisez l'électronique de pointe pour robotique. Affichages complexes, 
                    contrôle de servomoteurs et gestion énergétique avancée.
                  </p>

                  <div className="flex flex-wrap items-center justify-center gap-4 pt-4">
                    <div className="flex items-center gap-2 text-sm text-muted-foreground">
                      <Clock className="w-4 h-4" />
                      <span>~5.5h de contenu</span>
                    </div>
                    <div className="flex items-center gap-2 text-sm text-muted-foreground">
                      <BookOpen className="w-4 h-4" />
                      <span>2 sections expertes</span>
                    </div>
                    <div className="flex items-center gap-2 text-sm text-muted-foreground">
                      <Target className="w-4 h-4" />
                      <span>Niveau avancé</span>
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
                      Explorez les technologies électroniques de pointe pour systèmes robotiques experts
                    </p>
                  </div>

                  <div className="grid md:grid-cols-1 lg:grid-cols-2 gap-6">
                    {sections.map((section, index) => {
                      const Icon = section.icon
                      return (
                        <AnimatedSection key={section.title} animation="fade-up" delay={index * 100}>
                          <Card className="group hover:shadow-xl transition-all duration-300 border-2 hover:border-red-200 dark:hover:border-red-800">
                            <CardHeader className="pb-4">
                              <div className={`w-12 h-12 rounded-xl bg-gradient-to-r ${section.color} flex items-center justify-center mb-4 group-hover:scale-110 transition-transform duration-300`}>
                                <Icon className="w-6 h-6 text-white" />
                              </div>
                              <CardTitle className="text-xl group-hover:text-red-600 transition-colors">
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
                                <Button className="w-full group-hover:bg-red-600 group-hover:border-red-600 transition-colors">
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
                        <div className="w-6 h-6 rounded-full bg-red-100 dark:bg-red-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Monitor className="w-3 h-3 text-red-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Affichages multiplexés</h3>
                          <p className="text-sm text-muted-foreground">
                            Contrôle avancé d'afficheurs 7 segments et matrices LED
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-blue-100 dark:bg-blue-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Cpu className="w-3 h-3 text-blue-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Contrôle de servomoteurs</h3>
                          <p className="text-sm text-muted-foreground">
                            Positionnement précis et asservissement avancé
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="space-y-4">
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-green-100 dark:bg-green-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Battery className="w-3 h-3 text-green-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Gestion énergétique</h3>
                          <p className="text-sm text-muted-foreground">
                            Batteries lithium, BMS et optimisation de consommation
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-purple-100 dark:bg-purple-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Zap className="w-3 h-3 text-purple-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Systèmes haute performance</h3>
                          <p className="text-sm text-muted-foreground">
                            Intégration de composants avancés pour robotique professionnelle
                          </p>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>
        </main>
      </div>
    </div>
  )
}
