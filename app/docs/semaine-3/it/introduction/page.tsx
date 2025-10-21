"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import { Alert, AlertDescription } from "@/components/ui/alert";
import {
  ArrowRight, BookOpen, Code, FileText, Layers, Users, Info
} from "lucide-react";
import Link from "next/link";
import { useMounted } from "@/hooks/use-mounted";

export default function IntroductionPage() {
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
          {/* Header */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-16 bg-gradient-to-br from-purple-50 via-white to-pink-50 dark:from-purple-950/20 dark:via-background dark:to-pink-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-6">
                    <Link href="/docs/semaine-3" className="hover:text-foreground transition-colors">
                      Semaine 3
                    </Link>
                    <span>/</span>
                    <Link href="/docs/semaine-3/it" className="hover:text-foreground transition-colors">
                      IT
                    </Link>
                    <span>/</span>
                    <span>Introduction</span>
                  </div>

                  <div className="flex items-start gap-6 mb-8">
                    <div className="w-16 h-16 rounded-2xl bg-gradient-to-r from-purple-600 to-pink-600 flex items-center justify-center shadow-lg">
                      <BookOpen className="w-8 h-8 text-white" />
                    </div>
                    <div className="flex-1">
                      <h1 className="text-4xl md:text-5xl font-bold bg-gradient-to-r from-purple-600 to-pink-600 bg-clip-text text-transparent mb-4">
                        Introduction
                      </h1>
                      <p className="text-lg text-muted-foreground mb-6">
                        Introduction aux concepts avancés de l'informatique appliquée à la robotique et aux algorithmes de navigation
                      </p>
                      <div className="flex flex-wrap gap-2">
                        <Badge variant="secondary" className="bg-purple-100 text-purple-700 dark:bg-purple-900/20 dark:text-purple-300">
                          Avancé
                        </Badge>
                        <Badge variant="outline">Introduction</Badge>
                        <Badge variant="outline">Concepts</Badge>
                        <Badge variant="outline">Robotique</Badge>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
            <div className="container mx-auto px-4">
              <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
                <Link href="/docs/semaine-3/it">
                  <Button variant="ghost" size="sm">
                    <Code className="w-4 h-4 mr-2" />
                    Section IT
                  </Button>
                </Link>
                <Link href="/docs/semaine-3/it/algorithme-pathfinding">
                  <Button variant="ghost" size="sm">
                    Algorithme de Pathfinding
                    <ArrowRight className="w-4 h-4 ml-2" />
                  </Button>
                </Link>
              </div>
            </div>
          </div>

          {/* Content */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-5xl mx-auto">
              
              {/* Équipe et infos projet */}
              <AnimatedSection animation="fade-up">
                <Card className="border-l-4 border-l-purple-500 mb-8">
                  <CardHeader className="bg-gradient-to-r from-purple-50 to-transparent dark:from-purple-950/20">
                    <CardTitle className="flex items-center gap-2">
                      <Users className="w-5 h-5 text-purple-600" />
                      Équipe & Projet
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="pt-6">
                    <div className="grid md:grid-cols-2 gap-8">
                      <div>
                        <h3 className="text-lg font-semibold mb-4 text-gray-900 dark:text-gray-100">
                          Équipe IFRI IT
                        </h3>
                        <div className="space-y-3">
                          {[
                            "Axel HOUNSA",
                            "Bénédicte GANDJI", 
                            "Gérard DJOSSOU",
                            "Juvénal GNANGNON"
                          ].map((name, index) => (
                            <div key={index} className="flex items-center gap-3">
                              <div className="w-2 h-2 bg-purple-500 rounded-full flex-shrink-0"></div>
                              <span className="text-sm text-gray-700 dark:text-gray-300">{name}</span>
                            </div>
                          ))}
                        </div>
                      </div>
                      
                      <div>
                        <h3 className="text-lg font-semibold mb-4 text-gray-900 dark:text-gray-100">
                          Informations du projet
                        </h3>
                        <div className="space-y-4">
                          <div>
                            <div className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">Institution</div>
                            <div className="text-sm text-gray-600 dark:text-gray-400">
                              Institut de Formation et de Recherche en Informatique (IFRI)<br/>
                              Université d'Abomey-Calavi
                            </div>
                          </div>
                          <div>
                            <div className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">Date de réalisation</div>
                            <div className="text-sm text-gray-600 dark:text-gray-400">12 Juin 2025</div>
                          </div>
                          <div>
                            <div className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">Compétition</div>
                            <div className="text-sm text-gray-600 dark:text-gray-400">Tekbot Robotics Challenge 2025</div>
                          </div>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <Separator className="my-8" />

              {/* Alert d'information */}
              <AnimatedSection animation="fade-up" delay={25}>
                <Alert className="mb-8 border-purple-200 bg-purple-50 dark:border-purple-800 dark:bg-purple-950/20">
                  <Info className="h-4 w-4 text-purple-600" />
                  <AlertDescription className="text-purple-800 dark:text-purple-200">
                    <strong>Introduction IT Avancée :</strong> Cette section présente les concepts fondamentaux 
                    des algorithmes de navigation et de cartographie appliqués à la robotique.
                  </AlertDescription>
                </Alert>
              </AnimatedSection>

              {/* Contenu principal */}
              <AnimatedSection animation="fade-up" delay={50}>
                <Card className="border-l-4 border-l-pink-500">
                  <CardHeader className="bg-gradient-to-r from-pink-50 to-transparent dark:from-pink-950/20">
                    <CardTitle className="flex items-center gap-2">
                      <FileText className="w-5 h-5 text-pink-600" />
                      Documentation à venir
                    </CardTitle>
                    <CardDescription>
                      Concepts fondamentaux et introduction aux algorithmes avancés
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="pt-6">
                    <div className="text-center py-12">
                      <div className="w-16 h-16 bg-gradient-to-r from-purple-100 to-pink-100 dark:from-purple-900/20 dark:to-pink-900/20 rounded-2xl flex items-center justify-center mx-auto mb-4">
                        <Layers className="w-8 h-8 text-purple-600" />
                      </div>
                      <h3 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-2">
                        Contenu en préparation
                      </h3>
                      <p className="text-gray-600 dark:text-gray-400 max-w-md mx-auto">
                        L'introduction aux concepts avancés d'algorithmique et de robotique sera disponible prochainement.
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Navigation finale */}
              <AnimatedSection animation="fade-up" delay={75}>
                <div className="flex items-center justify-between pt-8 mt-8 border-t">
                  <Link href="/docs/semaine-3/it">
                    <Button variant="outline" className="group">
                      <Code className="w-4 h-4 mr-2" />
                      Section IT
                    </Button>
                  </Link>
                  <Link href="/docs/semaine-3/it/algorithme-pathfinding">
                    <Button className="group">
                      Algorithme Pathfinding
                      <ArrowRight className="w-4 h-4 ml-2 group-hover:translate-x-1 transition-transform" />
                    </Button>
                  </Link>
                </div>
              </AnimatedSection>
            </div>
          </div>
        </main>
      </div>
    </div>
  );
}
