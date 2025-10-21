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
  Shield
} from "lucide-react";
import Link from "next/link";

export default function Semaine2MecaniquePage() {
  return (
    <div className="min-h-screen bg-background" suppressHydrationWarning>
      <Navbar />
      
      <div className="flex pt-16 md:pt-20" suppressHydrationWarning>
        <DocsSidebarWrapper />
        
        <main className="flex-1 min-w-0" suppressHydrationWarning>
          {/* Hero Section */}
          <AnimatedSection animation="fade-up">
            <section className="relative py-20 bg-gradient-to-br from-blue-50 via-indigo-50 to-purple-50 dark:from-blue-900/10 dark:via-indigo-900/10 dark:to-purple-900/10">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto text-center">
                  <Badge variant="outline" className="mb-6 bg-white/50 dark:bg-gray-800/50">
                    <Shield className="w-4 h-4 mr-2" />
                    Semaine 2 - Mécanique
                  </Badge>
                  
                  <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6">
                    Tests Mécaniques
                    <span className="block bg-gradient-to-r from-blue-600 to-purple-600 bg-clip-text text-transparent">
                      Intermédiaires
                    </span>
                  </h1>
                  
                  <p className="text-xl text-muted-foreground max-w-3xl mx-auto mb-8">
                    Approfondissement des concepts mécaniques avec l'étude des contraintes complexes, 
                    l'analyse de résistance et la conception de systèmes robustes.
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
                      <Target className="w-5 h-5 text-blue-600" />
                      Objectifs du Test Mécanique
                    </CardTitle>
                    <CardDescription>
                      Maîtriser l'analyse des contraintes et la conception robuste
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <ul className="space-y-3">
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-blue-500 rounded-full mt-2"></div>
                        <span>Analyser les contraintes mécaniques complexes sur les structures</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-blue-500 rounded-full mt-2"></div>
                        <span>Calculer les coefficients de sécurité et facteurs de charge</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-blue-500 rounded-full mt-2"></div>
                        <span>Concevoir des assemblages résistants aux sollicitations</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-blue-500 rounded-full mt-2"></div>
                        <span>Optimiser la répartition des masses et contraintes</span>
                      </li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Clock className="w-5 h-5 text-indigo-600" />
                      Durée et Structure
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="grid md:grid-cols-2 gap-6">
                      <div>
                        <h4 className="font-semibold mb-2">Durée totale</h4>
                        <p className="text-muted-foreground">5 heures</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">Modalités</h4>
                        <p className="text-muted-foreground">Travail individuel avec logiciels CAO</p>
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
                      Consultez la documentation technique détaillée pour approfondir vos connaissances :
                    </p>
                    <Link href="/docs/semaine-2/mecanique/contraintes">
                      <Button variant="outline" className="w-full justify-start">
                        <Shield className="w-4 h-4 mr-2" />
                        Contraintes Complexes
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
                      <Wrench className="w-5 h-5 text-purple-600" />
                      Compétences Développées
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="grid md:grid-cols-2 gap-6">
                      <div className="space-y-3">
                        <h4 className="font-semibold text-sm">Analyse Structurelle</h4>
                        <ul className="text-sm text-muted-foreground space-y-1">
                          <li>• Diagrammes de corps libre</li>
                          <li>• Calcul des moments fléchissants</li>
                          <li>• Analyse des déformations</li>
                        </ul>
                      </div>
                      <div className="space-y-3">
                        <h4 className="font-semibold text-sm">Conception Avancée</h4>
                        <ul className="text-sm text-muted-foreground space-y-1">
                          <li>• Optimisation topologique</li>
                          <li>• Sélection des matériaux</li>
                          <li>• Validation par simulation</li>
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
                      Conseils pour Réussir
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-3">
                    <div className="text-sm space-y-2">
                      <p>• Maîtrisez les unités de contrainte (MPa, N/mm²)</p>
                      <p>• Vérifiez toujours les conditions d'équilibre statique</p>
                      <p>• Utilisez les logiciels de simulation pour valider vos calculs</p>
                      <p>• Prenez en compte les coefficients de sécurité appropriés</p>
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
                  <Link href="/docs/semaine-2/it">
                    <Button variant="outline" className="gap-2 group">
                      <ArrowLeft className="w-4 h-4 group-hover:-translate-x-1 transition-transform" />
                      IT Semaine 2
                    </Button>
                  </Link>
                  
                  <Link href="/docs/semaine-3">
                    <Button className="gap-2 group">
                      Semaine 3
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
