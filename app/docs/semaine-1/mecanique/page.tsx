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
  AlertCircle
} from "lucide-react";
import Link from "next/link";

export default function Semaine1MecaniquePage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />
      
      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />
        
        <main className="flex-1 min-w-0">
          {/* Hero Section */}
          <AnimatedSection animation="fade-up">
            <section className="relative py-20 bg-gradient-to-br from-orange-50 via-red-50 to-pink-50 dark:from-orange-900/10 dark:via-red-900/10 dark:to-pink-900/10">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto text-center">
                  <Badge variant="outline" className="mb-6 bg-white/50 dark:bg-gray-800/50">
                    <Wrench className="w-4 h-4 mr-2" />
                    Semaine 1 - Mécanique
                  </Badge>
                  
                  <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6">
                    Tests Mécaniques
                    <span className="block bg-gradient-to-r from-orange-600 to-red-600 bg-clip-text text-transparent">
                      Débutants
                    </span>
                  </h1>
                  
                  <p className="text-xl text-muted-foreground max-w-3xl mx-auto mb-8">
                    Introduction aux concepts fondamentaux de la mécanique robotique. 
                    Découvrez les bases de la conception assistée par ordinateur et les principes d'assemblage.
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
                      <Target className="w-5 h-5 text-orange-600" />
                      Objectifs du Test Mécanique
                    </CardTitle>
                    <CardDescription>
                      Maîtriser les fondamentaux de la conception mécanique
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <ul className="space-y-3">
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-orange-500 rounded-full mt-2"></div>
                        <span>Comprendre les principes de base de la CAO (Conception Assistée par Ordinateur)</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-orange-500 rounded-full mt-2"></div>
                        <span>Apprendre les techniques d'assemblage de pièces mécaniques</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-orange-500 rounded-full mt-2"></div>
                        <span>Réaliser des calculs de base pour la résistance des matériaux</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-orange-500 rounded-full mt-2"></div>
                        <span>Concevoir une pince mécanique fonctionnelle</span>
                      </li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Clock className="w-5 h-5 text-blue-600" />
                      Durée et Structure
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="grid md:grid-cols-2 gap-6">
                      <div>
                        <h4 className="font-semibold mb-2">Durée totale</h4>
                        <p className="text-muted-foreground">4 heures</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">Modalités</h4>
                        <p className="text-muted-foreground">Travail en binôme avec documentation autorisée</p>
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
                    <Link href="/docs/semaine-1/mecanique/cao">
                      <Button variant="outline" className="w-full justify-start">
                        <Wrench className="w-4 h-4 mr-2" />
                        Documentation CAO
                        <ArrowRight className="w-4 h-4 ml-auto" />
                      </Button>
                    </Link>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={300}>
                <Card className="border-yellow-200 bg-yellow-50/50 dark:border-yellow-800 dark:bg-yellow-900/10">
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2 text-yellow-800 dark:text-yellow-200">
                      <AlertCircle className="w-5 h-5" />
                      Conseils pour Réussir
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-3">
                    <div className="text-sm space-y-2">
                      <p>• Prenez le temps de bien comprendre les contraintes mécaniques</p>
                      <p>• Vérifiez vos calculs avant de valider votre conception</p>
                      <p>• Pensez à la faisabilité d'assemblage de vos pièces</p>
                      <p>• N'hésitez pas à itérer sur votre design initial</p>
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
                  <Link href="/docs/semaine-1/it">
                    <Button variant="outline" className="gap-2 group">
                      <ArrowLeft className="w-4 h-4 group-hover:-translate-x-1 transition-transform" />
                      IT Semaine 1
                    </Button>
                  </Link>
                  
                  <Link href="/docs/semaine-2">
                    <Button className="gap-2 group">
                      Semaine 2
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
