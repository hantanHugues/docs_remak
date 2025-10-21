"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowRight, BookOpen, Zap, Users, Target, Clock, CheckCircle, Star
} from "lucide-react";
import Link from "next/link";

export default function Semaine1Page() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* En-tête */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-16 bg-gradient-to-br from-green-50 via-white to-blue-50 dark:from-green-950/20 dark:via-background dark:to-blue-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-6">
                    <Link href="/pre-selection" className="hover:text-foreground transition-colors">
                      Pré-sélection
                    </Link>
                    <span>/</span>
                    <span>Semaine 1 - Débutants</span>
                  </div>

                  <div className="flex items-start gap-6 mb-8">
                    <div className="w-16 h-16 rounded-2xl bg-gradient-to-r from-green-600 to-blue-600 flex items-center justify-center shadow-lg">
                      <Star className="w-8 h-8 text-white" />
                    </div>
                    <div>
                      <Badge className="mb-4 bg-green-100 text-green-800 hover:bg-green-200 dark:bg-green-900 dark:text-green-100">
                        Niveau Débutant
                      </Badge>
                      <h1 className="text-4xl md:text-5xl font-bold mb-4 bg-gradient-to-r from-green-600 to-blue-600 bg-clip-text text-transparent">
                        Semaine 1 - Débutants
                      </h1>
                      <p className="text-xl text-muted-foreground leading-relaxed">
                        Découvrez les fondamentaux de la robotique à travers trois domaines essentiels : l'électronique, l'informatique et la mécanique.
                      </p>
                    </div>
                  </div>

                  <div className="flex flex-wrap gap-3">
                    <Badge variant="secondary" className="flex items-center gap-1">
                      <Clock className="w-3 h-3" />
                      7 jours
                    </Badge>
                    <Badge variant="outline">Arduino</Badge>
                    <Badge variant="outline">Capteurs</Badge>
                    <Badge variant="outline">Programmation</Badge>
                    <Badge variant="outline">CAO</Badge>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Contenu */}
          <div className="container mx-auto px-4 py-12">
            <div className="max-w-4xl mx-auto space-y-12">
              
              {/* Objectifs */}
              <AnimatedSection animation="fade-up">
                <Card className="border-l-4 border-l-green-500">
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2 text-green-700 dark:text-green-300">
                      <Target className="w-5 h-5" />
                      Objectifs de la semaine
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-3">
                    <div className="flex items-start gap-3">
                      <CheckCircle className="w-5 h-5 text-green-500 mt-0.5 flex-shrink-0" />
                      <p>Maîtriser les bases de l'électronique avec Arduino et les capteurs</p>
                    </div>
                    <div className="flex items-start gap-3">
                      <CheckCircle className="w-5 h-5 text-green-500 mt-0.5 flex-shrink-0" />
                      <p>Comprendre la programmation orientée objet appliquée à la robotique</p>
                    </div>
                    <div className="flex items-start gap-3">
                      <CheckCircle className="w-5 h-5 text-green-500 mt-0.5 flex-shrink-0" />
                      <p>Découvrir la conception mécanique et la CAO</p>
                    </div>
                    <div className="flex items-start gap-3">
                      <CheckCircle className="w-5 h-5 text-green-500 mt-0.5 flex-shrink-0" />
                      <p>Réaliser votre premier projet robotique intégré</p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <Separator />

              {/* Domaines */}
              <div className="grid md:grid-cols-3 gap-6">
                
                {/* Électronique */}
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="h-full hover:shadow-lg transition-shadow duration-300 group">
                    <CardHeader className="text-center">
                      <div className="w-12 h-12 rounded-lg bg-yellow-100 dark:bg-yellow-900/20 mx-auto mb-4 flex items-center justify-center group-hover:scale-110 transition-transform duration-300">
                        <Zap className="w-6 h-6 text-yellow-600 dark:text-yellow-400" />
                      </div>
                      <CardTitle className="text-yellow-700 dark:text-yellow-300">Électronique</CardTitle>
                      <CardDescription>
                        Découvrez Arduino, les capteurs et les circuits de base
                      </CardDescription>
                    </CardHeader>
                    <CardContent className="space-y-3">
                      <div className="space-y-2 text-sm">
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-yellow-500 rounded-full"></div>
                          <span>Gyroscope & Accéléromètre</span>
                        </div>
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-yellow-500 rounded-full"></div>
                          <span>Circuit I2C</span>
                        </div>
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-yellow-500 rounded-full"></div>
                          <span>Affichage LCD</span>
                        </div>
                      </div>
                      <Button asChild className="w-full mt-4">
                        <Link href="/docs/semaine-1/electronique">
                          Explorer <ArrowRight className="w-4 h-4 ml-1" />
                        </Link>
                      </Button>
                    </CardContent>
                  </Card>
                </AnimatedSection>

                {/* IT */}
                <AnimatedSection animation="fade-up" delay={200}>
                  <Card className="h-full hover:shadow-lg transition-shadow duration-300 group">
                    <CardHeader className="text-center">
                      <div className="w-12 h-12 rounded-lg bg-blue-100 dark:bg-blue-900/20 mx-auto mb-4 flex items-center justify-center group-hover:scale-110 transition-transform duration-300">
                        <BookOpen className="w-6 h-6 text-blue-600 dark:text-blue-400" />
                      </div>
                      <CardTitle className="text-blue-700 dark:text-blue-300">Informatique</CardTitle>
                      <CardDescription>
                        Programmation orientée objet et robotique
                      </CardDescription>
                    </CardHeader>
                    <CardContent className="space-y-3">
                      <div className="space-y-2 text-sm">
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-blue-500 rounded-full"></div>
                          <span>Classe Robot</span>
                        </div>
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-blue-500 rounded-full"></div>
                          <span>Bras Robotique</span>
                        </div>
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-blue-500 rounded-full"></div>
                          <span>Robot à Roues</span>
                        </div>
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-blue-500 rounded-full"></div>
                          <span>Diagrammes UML</span>
                        </div>
                      </div>
                      <Button asChild className="w-full mt-4">
                        <Link href="/docs/semaine-1/it">
                          Explorer <ArrowRight className="w-4 h-4 ml-1" />
                        </Link>
                      </Button>
                    </CardContent>
                  </Card>
                </AnimatedSection>

                {/* Mécanique */}
                <AnimatedSection animation="fade-up" delay={300}>
                  <Card className="h-full hover:shadow-lg transition-shadow duration-300 group">
                    <CardHeader className="text-center">
                      <div className="w-12 h-12 rounded-lg bg-purple-100 dark:bg-purple-900/20 mx-auto mb-4 flex items-center justify-center group-hover:scale-110 transition-transform duration-300">
                        <Users className="w-6 h-6 text-purple-600 dark:text-purple-400" />
                      </div>
                      <CardTitle className="text-purple-700 dark:text-purple-300">Mécanique</CardTitle>
                      <CardDescription>
                        Conception CAO et assemblage mécanique
                      </CardDescription>
                    </CardHeader>
                    <CardContent className="space-y-3">
                      <div className="space-y-2 text-sm">
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-purple-500 rounded-full"></div>
                          <span>Documentation CAO</span>
                        </div>
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-purple-500 rounded-full"></div>
                          <span>Pièces d'assemblage</span>
                        </div>
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-purple-500 rounded-full"></div>
                          <span>Pince mécanique</span>
                        </div>
                        <div className="flex items-center gap-2">
                          <div className="w-2 h-2 bg-purple-500 rounded-full"></div>
                          <span>Calculs de masse</span>
                        </div>
                      </div>
                      <Button asChild className="w-full mt-4">
                        <Link href="/docs/semaine-1/mecanique">
                          Explorer <ArrowRight className="w-4 h-4 ml-1" />
                        </Link>
                      </Button>
                    </CardContent>
                  </Card>
                </AnimatedSection>
              </div>

              {/* Navigation */}
              <AnimatedSection animation="fade-up" delay={400}>
                <div className="flex justify-between items-center pt-8 border-t">
                  <Link href="/pre-selection">
                    <Button variant="outline">
                      Retour à la Pré-sélection
                    </Button>
                  </Link>
                  <Link href="/docs/semaine-2">
                    <Button>
                      Semaine 2 - Intermédiaires
                      <ArrowRight className="w-4 h-4 ml-2" />
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
