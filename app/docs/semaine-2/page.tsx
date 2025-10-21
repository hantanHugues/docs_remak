"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Cpu, Code, Cog, ArrowRight, ArrowLeft, BookOpen, Target, Clock } from "lucide-react";
import Link from "next/link";

export default function Semaine2Page() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          <AnimatedSection animation="fade-up">
            <section className="py-20 px-4">
              <div className="container mx-auto max-w-4xl">
                <div className="text-center mb-16">
                  <Badge variant="outline" className="mb-4">
                    Semaine 2 - Intermédiaire
                  </Badge>
                  <h1 className="text-4xl font-bold tracking-tight mb-6">
                    Documentation Technique
                    <span className="block text-blue-600">Semaine 2</span>
                  </h1>
                  <p className="text-xl text-muted-foreground max-w-2xl mx-auto">
                    Approfondissement des concepts techniques avec des projets intégrés
                    et des défis de niveau intermédiaire.
                  </p>
                </div>

                <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                  <Card className="group hover:shadow-lg transition-all duration-300">
                    <CardHeader>
                      <div className="w-12 h-12 bg-blue-100 dark:bg-blue-900/20 rounded-lg flex items-center justify-center mb-4">
                        <Cpu className="w-6 h-6 text-blue-600" />
                      </div>
                      <CardTitle>Électronique</CardTitle>
                      <CardDescription>
                        Systèmes électroniques complexes et intégration
                      </CardDescription>
                    </CardHeader>
                    <CardContent>
                      <ul className="space-y-2 text-sm mb-4">
                        <li>• Boîte noire & station de contrôle</li>
                        <li>• Hardware avancé</li>
                        <li>• PCB et transmission</li>
                        <li>• Troubleshooting système</li>
                      </ul>
                      <Link href="/docs/semaine-2/electronique">
                        <Button className="w-full group-hover:bg-blue-600">
                          Commencer
                          <ArrowRight className="w-4 h-4 ml-2" />
                        </Button>
                      </Link>
                    </CardContent>
                  </Card>

                  <Card className="group hover:shadow-lg transition-all duration-300">
                    <CardHeader>
                      <div className="w-12 h-12 bg-green-100 dark:bg-green-900/20 rounded-lg flex items-center justify-center mb-4">
                        <Code className="w-6 h-6 text-green-600" />
                      </div>
                      <CardTitle>IT</CardTitle>
                      <CardDescription>
                        ROS2, capteurs et développement d'interfaces
                      </CardDescription>
                    </CardHeader>
                    <CardContent>
                      <ul className="space-y-2 text-sm mb-4">
                        <li>• Évaluation des capteurs ROS2</li>
                        <li>• Configuration de lancement</li>
                        <li>• Publisher/Subscriber</li>
                        <li>• Dashboard Streamlit</li>
                      </ul>
                      <Link href="/docs/semaine-2/it">
                        <Button className="w-full group-hover:bg-green-600">
                          Commencer
                          <ArrowRight className="w-4 h-4 ml-2" />
                        </Button>
                      </Link>
                    </CardContent>
                  </Card>

                  <Card className="group hover:shadow-lg transition-all duration-300">
                    <CardHeader>
                      <div className="w-12 h-12 bg-orange-100 dark:bg-orange-900/20 rounded-lg flex items-center justify-center mb-4">
                        <Cog className="w-6 h-6 text-orange-600" />
                      </div>
                      <CardTitle>Mécanique</CardTitle>
                      <CardDescription>
                        Conception avancée et optimisation
                      </CardDescription>
                    </CardHeader>
                    <CardContent>
                      <ul className="space-y-2 text-sm mb-4">
                        <li>• Conception paramétrique</li>
                        <li>• Assemblages complexes</li>
                        <li>• Simulations mécaniques</li>
                        <li>• Validation prototype</li>
                      </ul>
                      <Link href="/docs/semaine-2/mecanique">
                        <Button className="w-full group-hover:bg-orange-600" disabled>
                          Bientôt disponible
                          <Clock className="w-4 h-4 ml-2" />
                        </Button>
                      </Link>
                    </CardContent>
                  </Card>
                </div>

                <div className="mt-16 text-center">
                  <div className="bg-muted/50 rounded-lg p-8">
                    <BookOpen className="w-12 h-12 text-muted-foreground mx-auto mb-4" />
                    <h3 className="text-lg font-semibold mb-2">Objectifs de la Semaine 2</h3>
                    <p className="text-muted-foreground max-w-2xl mx-auto">
                      Maîtriser l'intégration de systèmes complexes, développer des compétences
                      en debugging et en optimisation, et créer des solutions techniques robustes.
                    </p>
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
                  <Link href="/docs/semaine-1">
                    <Button variant="outline" className="gap-2">
                      <ArrowLeft className="w-4 h-4" />
                      Semaine 1
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
