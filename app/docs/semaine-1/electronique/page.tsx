"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Cpu, Zap, Wifi, Monitor, ArrowRight, BookOpen, Target, Clock } from "lucide-react"
import Link from "next/link"


const sections = [
  {
    title: "Gyroscope & Accéléromètre",
    description: "Comprendre et utiliser les capteurs de mouvement pour la robotique",
    href: "/docs/semaine-1/electronique/gyroscope",
    icon: Target,
    difficulty: "Débutant",
    duration: "2h",
    color: "from-blue-500 to-cyan-500"
  },
  {
    title: "Circuit I2C",
    description: "Maîtriser la communication I2C entre composants électroniques",
    href: "/docs/semaine-1/electronique/i2c",
    icon: Wifi,
    difficulty: "Débutant",
    duration: "1.5h",
    color: "from-green-500 to-emerald-500"
  },
  {
    title: "Affichage LCD",
    description: "Intégrer et contrôler des écrans LCD dans vos projets",
    href: "/docs/semaine-1/electronique/lcd",
    icon: Monitor,
    difficulty: "Débutant",
    duration: "1h",
    color: "from-purple-500 to-pink-500"
  }
]

export default function ElectroniquePage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* Hero Section */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-20 overflow-hidden bg-gradient-to-br from-blue-50 via-white to-cyan-50 dark:from-blue-950/20 dark:via-background dark:to-cyan-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto text-center space-y-6">
                  <Badge className="bg-blue-100 text-blue-700 border-blue-200 dark:bg-blue-900/50 dark:text-blue-300 dark:border-blue-800">
                    <Cpu className="w-4 h-4 mr-2" />
                    Semaine 1 - Électronique
                  </Badge>
                  
                  <h1 className="text-4xl md:text-6xl font-bold text-foreground">
                    Électronique <span className="text-blue-600">Débutant</span>
                  </h1>
                  
                  <p className="text-xl text-muted-foreground max-w-2xl mx-auto leading-relaxed">
                    Découvrez les fondamentaux de l'électronique appliquée à la robotique. 
                    Apprenez à utiliser les capteurs, la communication I2C et les affichages.
                  </p>

                  <div className="flex flex-wrap items-center justify-center gap-4 pt-4">
                    <div className="flex items-center gap-2 text-sm text-muted-foreground">
                      <Clock className="w-4 h-4" />
                      <span>~4.5h de contenu</span>
                    </div>
                    <div className="flex items-center gap-2 text-sm text-muted-foreground">
                      <BookOpen className="w-4 h-4" />
                      <span>3 sections pratiques</span>
                    </div>
                    <div className="flex items-center gap-2 text-sm text-muted-foreground">
                      <Target className="w-4 h-4" />
                      <span>Niveau débutant</span>
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
                      Explorez chaque section pour maîtriser les concepts essentiels de l'électronique robotique
                    </p>
                  </div>

                  <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                    {sections.map((section, index) => {
                      const Icon = section.icon
                      return (
                        <AnimatedSection key={section.title} animation="fade-up" delay={index * 100}>
                          <Card className="group hover:shadow-xl transition-all duration-300 border-2 hover:border-blue-200 dark:hover:border-blue-800">
                            <CardHeader className="pb-4">
                              <div className={`w-12 h-12 rounded-xl bg-gradient-to-r ${section.color} flex items-center justify-center mb-4 group-hover:scale-110 transition-transform duration-300`}>
                                <Icon className="w-6 h-6 text-white" />
                              </div>
                              <CardTitle className="text-xl group-hover:text-blue-600 transition-colors">
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
                                <Button className="w-full group-hover:bg-blue-600 group-hover:border-blue-600 transition-colors">
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
                        <div className="w-6 h-6 rounded-full bg-blue-100 dark:bg-blue-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Zap className="w-3 h-3 text-blue-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Capteurs de mouvement</h3>
                          <p className="text-sm text-muted-foreground">
                            Utilisation des gyroscopes et accéléromètres pour détecter l'orientation et le mouvement
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-green-100 dark:bg-green-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Wifi className="w-3 h-3 text-green-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Communication I2C</h3>
                          <p className="text-sm text-muted-foreground">
                            Protocole de communication série pour connecter plusieurs composants
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="space-y-4">
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-purple-100 dark:bg-purple-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Monitor className="w-3 h-3 text-purple-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Affichage LCD</h3>
                          <p className="text-sm text-muted-foreground">
                            Intégration et contrôle d'écrans LCD pour l'interface utilisateur
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-orange-100 dark:bg-orange-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <Target className="w-3 h-3 text-orange-600" />
                        </div>
                        <div>
                          <h3 className="font-semibold mb-1">Applications pratiques</h3>
                          <p className="text-sm text-muted-foreground">
                            Mise en œuvre dans des projets robotiques concrets
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
