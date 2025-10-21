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
  Target, 
  Trophy,
  Clock,
  BookOpen,
  AlertCircle,
  Zap
} from "lucide-react";
import Link from "next/link";

export default function TestFinalPage() {
  const sections = [
    {
      title: "Électronique",
      description: "Systèmes de contrôle et capteurs pour le convoyeur de tri",
      href: "/docs/test-final/electronique",
      difficulty: "Expert",
      duration: "6h",
      color: "from-blue-500 to-indigo-500",
      topics: ["Capteurs de couleur", "Systèmes embarqués", "PCB personnalisés"]
    },
    {
      title: "Informatique",
      description: "Dashboard industriel et algorithmes de classification intelligente",
      href: "/docs/test-final/it",
      difficulty: "Expert", 
      duration: "8h",
      color: "from-green-500 to-emerald-500",
      topics: ["Classification couleurs", "Dashboard temps réel", "Machine Learning"]
    },
    {
      title: "Mécanique",
      description: "Conception et assemblage du système de convoyeur",
      href: "/docs/test-final/mecanique",
      difficulty: "Expert",
      duration: "4h", 
      color: "from-purple-500 to-violet-500",
      topics: ["Convoyeur industriel", "Documentation finale"]
    }
  ];

  return (
    <div className="min-h-screen bg-background" suppressHydrationWarning>
      <Navbar />
      
      <div className="flex pt-16 md:pt-20" suppressHydrationWarning>
        <DocsSidebarWrapper />
        
        <main className="flex-1 min-w-0" suppressHydrationWarning>
          {/* Hero Section */}
          <AnimatedSection animation="fade-up">
            <section className="relative py-20 bg-gradient-to-br from-yellow-50 via-orange-50 to-red-50 dark:from-yellow-900/10 dark:via-orange-900/10 dark:to-red-900/10">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto text-center">
                  <Badge variant="outline" className="mb-6 bg-white/50 dark:bg-gray-800/50">
                    <Trophy className="w-4 h-4 mr-2" />
                    Test Final - Évaluation Complète
                  </Badge>
                  
                  <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6">
                    Projet de
                    <span className="block bg-gradient-to-r from-orange-600 to-red-600 bg-clip-text text-transparent">
                      Test Final
                    </span>
                  </h1>
                  
                  <p className="text-xl text-muted-foreground max-w-3xl mx-auto mb-8">
                    Projet intégrateur combinant électronique, informatique et mécanique 
                    pour créer un système de tri automatisé complet avec convoyeur industriel.
                  </p>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Sections Grid */}
          <div className="container mx-auto px-4 py-20">
            <div className="max-w-6xl mx-auto">
              <AnimatedSection animation="fade-up">
                <div className="text-center mb-12">
                  <h2 className="text-3xl font-bold mb-4">Épreuve Multidisciplinaire</h2>
                  <p className="text-muted-foreground max-w-2xl mx-auto">
                    Un défi technique complet intégrant les trois domaines d'expertise 
                    pour créer un système industriel fonctionnel.
                  </p>
                </div>
              </AnimatedSection>

              <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8">
                {sections.map((section, index) => (
                  <AnimatedSection key={index} animation="fade-up" delay={index * 150}>
                    <Link href={section.href}>
                      <Card className="group hover:shadow-xl transition-all duration-300 border-0 bg-gradient-to-br from-white to-gray-50/50 dark:from-gray-800 dark:to-gray-900/50 overflow-hidden h-full">
                        <CardHeader className="pb-4">
                          <div className={`w-12 h-12 rounded-lg bg-gradient-to-r ${section.color} p-2.5 mb-4 group-hover:scale-110 transition-transform duration-300`}>
                            <Target className="w-full h-full text-white" />
                          </div>
                          
                          <div className="flex items-center justify-between mb-2">
                            <Badge variant="secondary" className="text-xs">
                              {section.difficulty}
                            </Badge>
                            <Badge variant="outline" className="text-xs">
                              {section.duration}
                            </Badge>
                          </div>
                          
                          <CardTitle className="group-hover:text-orange-600 transition-colors">
                            {section.title}
                          </CardTitle>
                          
                          <CardDescription className="text-sm">
                            {section.description}
                          </CardDescription>
                        </CardHeader>
                        
                        <CardContent>
                          <div className="space-y-2">
                            <div className="text-sm font-medium text-muted-foreground mb-2">
                              Technologies clés :
                            </div>
                            <div className="flex flex-wrap gap-2">
                              {section.topics.map((topic, topicIndex) => (
                                <Badge key={topicIndex} variant="outline" className="text-xs">
                                  {topic}
                                </Badge>
                              ))}
                            </div>
                          </div>
                          
                          <div className="flex items-center text-orange-600 mt-4 group-hover:translate-x-1 transition-transform">
                            <span className="text-sm font-medium">Consulter</span>
                            <ArrowRight className="w-4 h-4 ml-1" />
                          </div>
                        </CardContent>
                      </Card>
                    </Link>
                  </AnimatedSection>
                ))}
              </div>
            </div>
          </div>

          {/* Objectifs Section */}
          <AnimatedSection animation="fade-up">
            <section className="py-20 bg-gradient-to-r from-orange-50 to-red-50 dark:from-orange-900/10 dark:to-red-900/10">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto text-center">
                  <div className="w-16 h-16 bg-gradient-to-r from-orange-600 to-red-600 rounded-2xl flex items-center justify-center mx-auto mb-6">
                    <Trophy className="w-8 h-8 text-white" />
                  </div>
                  
                  <h3 className="text-2xl font-bold mb-4">Objectif du Test Final</h3>
                  
                  <p className="text-lg text-muted-foreground mb-8 max-w-3xl mx-auto">
                    Concevoir et réaliser un système complet de tri automatisé par convoyeur, 
                    intégrant reconnaissance visuelle, contrôle temps réel et supervision industrielle.
                  </p>

                  <div className="grid md:grid-cols-3 gap-6 mt-12">
                    <div className="text-center">
                      <div className="w-12 h-12 bg-blue-100 dark:bg-blue-900/50 rounded-lg flex items-center justify-center mx-auto mb-3">
                        <Zap className="w-6 h-6 text-blue-600" />
                      </div>
                      <h4 className="font-semibold mb-2">Intégration Complète</h4>
                      <p className="text-sm text-muted-foreground">
                        Fusion des trois disciplines pour un système industriel
                      </p>
                    </div>
                    
                    <div className="text-center">
                      <div className="w-12 h-12 bg-green-100 dark:bg-green-900/50 rounded-lg flex items-center justify-center mx-auto mb-3">
                        <Target className="w-6 h-6 text-green-600" />
                      </div>
                      <h4 className="font-semibold mb-2">Performance Optimale</h4>
                      <p className="text-sm text-muted-foreground">
                        Précision et efficacité dans le tri automatisé
                      </p>
                    </div>
                    
                    <div className="text-center">
                      <div className="w-12 h-12 bg-purple-100 dark:bg-purple-900/50 rounded-lg flex items-center justify-center mx-auto mb-3">
                        <BookOpen className="w-6 h-6 text-purple-600" />
                      </div>
                      <h4 className="font-semibold mb-2">Documentation Industrielle</h4>
                      <p className="text-sm text-muted-foreground">
                        Rapport technique complet et professionnel
                      </p>
                    </div>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="container mx-auto px-4 py-12">
            <div className="max-w-4xl mx-auto">
              <AnimatedSection animation="fade-up">
                <div className="flex justify-between items-center pt-8 border-t">
                  <Link href="/docs/semaine-3">
                    <Button variant="outline" className="gap-2 group">
                      <ArrowLeft className="w-4 h-4 group-hover:-translate-x-1 transition-transform" />
                      Semaine 3
                    </Button>
                  </Link>
                  
                  <Link href="/finale">
                    <Button className="gap-2 group bg-gradient-to-r from-orange-600 to-red-600 hover:from-orange-700 hover:to-red-700">
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
