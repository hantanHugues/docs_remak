"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { 
  ArrowLeft, 
  ArrowRight, 
  Wrench,
  Target, 
  Clock,
  BookOpen,
  AlertCircle,
  Settings
} from "lucide-react";
import Link from "next/link";

export default function TestFinalMecaniquePage() {
  return (
    <div className="min-h-screen bg-background" suppressHydrationWarning>
      <Navbar />
      
      <div className="flex pt-16 md:pt-20" suppressHydrationWarning>
        <DocsSidebarWrapper />
        
        <main className="flex-1 min-w-0" suppressHydrationWarning>
          {/* Hero Section */}
          <AnimatedSection animation="fade-up">
            <section className="relative py-20 bg-gradient-to-br from-purple-50 via-violet-50 to-pink-50 dark:from-purple-900/10 dark:via-violet-900/10 dark:to-pink-900/10">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto text-center">
                  <Badge variant="outline" className="mb-6 bg-white/50 dark:bg-gray-800/50">
                    <Settings className="w-4 h-4 mr-2" />
                    Test Final - Mécanique
                  </Badge>
                  
                  <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6">
                    Système de Convoyeur
                    <span className="block bg-gradient-to-r from-purple-600 to-violet-600 bg-clip-text text-transparent">
                      Industriel
                    </span>
                  </h1>
                  
                  <p className="text-xl text-muted-foreground max-w-3xl mx-auto mb-8">
                    Conception, assemblage et maintenance du système de convoyeur 
                    pour le tri automatisé industriel avec documentation technique complète.
                  </p>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Contenu principal */}
          <div className="container mx-auto px-4 py-20">
            <div className="max-w-4xl mx-auto space-y-12">
              
              <AnimatedSection animation="fade-up">
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Target className="w-5 h-5 text-purple-600" />
                      Objectifs Mécaniques
                    </CardTitle>
                    <CardDescription>
                      Conception et réalisation d'un système de convoyeur industriel
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <ul className="space-y-3">
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-purple-500 rounded-full mt-2"></div>
                        <span>Concevoir la structure du convoyeur et ses supports</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-purple-500 rounded-full mt-2"></div>
                        <span>Dimensionner le système de tapis roulant</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-purple-500 rounded-full mt-2"></div>
                        <span>Intégrer les systèmes d'entraînement et de guidage</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-purple-500 rounded-full mt-2"></div>
                        <span>Assurer la maintenance et la documentation</span>
                      </li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Clock className="w-5 h-5 text-violet-600" />
                      Composants Mécaniques
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="grid md:grid-cols-2 gap-6">
                      <div>
                        <h4 className="font-semibold mb-2">Structure</h4>
                        <p className="text-muted-foreground text-sm">Châssis, supports, guides</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">Tapis</h4>
                        <p className="text-muted-foreground text-sm">Bande transporteuse, rouleaux</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">Entraînement</h4>
                        <p className="text-muted-foreground text-sm">Moteurs, réducteurs, courroies</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">Assemblage</h4>
                        <p className="text-muted-foreground text-sm">Fixations, alignements</p>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={200}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <BookOpen className="w-5 h-5 text-green-600" />
                      Documentation Technique
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <p className="text-muted-foreground mb-4">
                      Consultez la documentation technique complète du système mécanique :
                    </p>
                    <Link href="/docs/test-final/mecanique/documentation-finale">
                      <Button variant="outline" className="w-full justify-start">
                        <Wrench className="w-4 h-4 mr-2" />
                        Documentation Finale Complète
                        <ArrowRight className="w-4 h-4 ml-auto" />
                      </Button>
                    </Link>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={300}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Settings className="w-5 h-5 text-blue-600" />
                      Spécifications Techniques
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="grid md:grid-cols-2 gap-6">
                      <div className="space-y-3">
                        <h4 className="font-semibold text-sm">Dimensions</h4>
                        <ul className="text-sm text-muted-foreground space-y-1">
                          <li>• Longueur : Variable selon besoins</li>
                          <li>• Largeur : Standard industriel</li>
                          <li>• Hauteur : Réglable</li>
                        </ul>
                      </div>
                      <div className="space-y-3">
                        <h4 className="font-semibold text-sm">Performance</h4>
                        <ul className="text-sm text-muted-foreground space-y-1">
                          <li>• Vitesse réglable</li>
                          <li>• Charge max : Définie</li>
                          <li>• Précision de positionnement</li>
                        </ul>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={400}>
                <Card className="border-yellow-200 bg-yellow-50/50 dark:border-yellow-800 dark:bg-yellow-900/10">
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2 text-yellow-800 dark:text-yellow-200">
                      <AlertCircle className="w-5 h-5" />
                      Points Clés Mécaniques
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-3">
                    <div className="text-sm space-y-2">
                      <p>• Robustesse et fiabilité du système</p>
                      <p>• Facilité de maintenance et d'accès</p>
                      <p>• Intégration avec les systèmes électroniques</p>
                      <p>• Respect des normes industrielles</p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
            </div>
          </div>

          {/* Navigation */}
          <div className="container mx-auto px-4 py-12">
            <div className="max-w-4xl mx-auto">
              <AnimatedSection animation="fade-up">
                <div className="flex justify-between items-center pt-8 border-t">
                  <Link href="/docs/test-final/it">
                    <Button variant="outline" className="gap-2 group">
                      <ArrowLeft className="w-4 h-4 group-hover:-translate-x-1 transition-transform" />
                      IT Test Final
                    </Button>
                  </Link>
                  
                  <Link href="/finale">
                    <Button className="gap-2 group">
                      Phase Finale
                      <ArrowRight className="w-4 h-4 group-hover:translate-x-1 transition-transform" />
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
