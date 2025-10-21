"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Cpu, Code, Cog, ArrowRight, ArrowLeft, Trophy } from "lucide-react";
import Link from "next/link";
import { useMounted } from "@/hooks/use-mounted";

export default function Semaine3Page() {
  const mounted = useMounted();

  if (!mounted) {
    return null;
  }

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
                    Semaine 3 - Avancé
                  </Badge>
                  <h1 className="text-4xl font-bold tracking-tight mb-6">
                    Documentation Technique
                    <span className="block text-purple-600">Semaine 3</span>
                  </h1>
                  <p className="text-xl text-muted-foreground max-w-2xl mx-auto">
                    Projets avancés, algorithmes sophistiqués et défis techniques
                    de niveau expert pour la préparation finale.
                  </p>
                </div>

                <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                  <Card className="group hover:shadow-lg transition-all duration-300">
                    <CardHeader>
                      <div className="w-12 h-12 bg-purple-100 dark:bg-purple-900/20 rounded-lg flex items-center justify-center mb-4">
                        <Cpu className="w-6 h-6 text-purple-600" />
                      </div>
                      <CardTitle>Électronique</CardTitle>
                      <CardDescription>
                        Systèmes électroniques de pointe et innovations
                      </CardDescription>
                    </CardHeader>
                    <CardContent>
                      <ul className="space-y-2 text-sm mb-4">
                        <li>• Afficheurs 7 segments</li>
                        <li>• Servomoteurs avancés</li>
                        <li>• Programmation embarquée</li>
                        <li>• Optimisation système</li>
                      </ul>
                      <Link href="/docs/semaine-3/electronique">
                        <Button className="w-full group-hover:bg-purple-600">
                          Commencer
                          <ArrowRight className="w-4 h-4 ml-2" />
                        </Button>
                      </Link>
                    </CardContent>
                  </Card>

                  <Card className="group hover:shadow-lg transition-all duration-300">
                    <CardHeader>
                      <div className="w-12 h-12 bg-indigo-100 dark:bg-indigo-900/20 rounded-lg flex items-center justify-center mb-4">
                        <Code className="w-6 h-6 text-indigo-600" />
                      </div>
                      <CardTitle>IT</CardTitle>
                      <CardDescription>
                        Intelligence artificielle et algorithmes complexes
                      </CardDescription>
                    </CardHeader>
                    <CardContent>
                      <ul className="space-y-2 text-sm mb-4">
                        <li>• Introduction aux algorithmes</li>
                        <li>• Pathfinding avancé</li>
                        <li>• Cartographie & Nav2</li>
                        <li>• IA pour robotique</li>
                      </ul>
                      <Link href="/docs/semaine-3/it">
                        <Button className="w-full group-hover:bg-indigo-600">
                          Commencer
                          <ArrowRight className="w-4 h-4 ml-2" />
                        </Button>
                      </Link>
                    </CardContent>
                  </Card>

                  <Card className="group hover:shadow-lg transition-all duration-300">
                    <CardHeader>
                      <div className="w-12 h-12 bg-emerald-100 dark:bg-emerald-900/20 rounded-lg flex items-center justify-center mb-4">
                        <Cog className="w-6 h-6 text-emerald-600" />
                      </div>
                      <CardTitle>Mécanique</CardTitle>
                      <CardDescription>
                        Ingénierie de précision et innovation
                      </CardDescription>
                    </CardHeader>
                    <CardContent>
                      <ul className="space-y-2 text-sm mb-4">
                        <li>• Paramètres variables</li>
                        <li>• Impression 3D avancée</li>
                        <li>• Calculs de précision</li>
                        <li>• Prototypage rapide</li>
                      </ul>
                      <Button className="w-full" disabled>
                        Bientôt disponible
                      </Button>
                    </CardContent>
                  </Card>
                </div>

                <div className="mt-16 text-center">
                  <div className="bg-gradient-to-r from-purple-50 to-indigo-50 dark:from-purple-900/10 dark:to-indigo-900/10 rounded-lg p-8">
                    <Trophy className="w-12 h-12 text-purple-600 mx-auto mb-4" />
                    <h3 className="text-lg font-semibold mb-2">Objectifs de la Semaine 3</h3>
                    <p className="text-muted-foreground max-w-2xl mx-auto">
                      Maîtriser les algorithmes avancés, développer des systèmes autonomes
                      et préparer les projets finaux pour la compétition.
                    </p>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          <div className="container mx-auto px-4 py-8">
            <div className="max-w-4xl mx-auto">
              <AnimatedSection animation="fade-up">
                <div className="flex justify-between items-center pt-8 border-t">
                  <Link href="/docs/semaine-2">
                    <Button variant="outline" className="gap-2">
                      <ArrowLeft className="w-4 h-4" />
                      Semaine 2
                    </Button>
                  </Link>
                  <Link href="/finale">
                    <Button className="gap-2">
                      Test Final
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