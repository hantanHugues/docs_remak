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
  Cpu,
  Target, 
  Clock,
  BookOpen,
  AlertCircle
} from "lucide-react";
import Link from "next/link";

export default function TestFinalElectroniquePage() {
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
                    <Cpu className="w-4 h-4 mr-2" />
                    Test Final - Électronique
                  </Badge>
                  
                  <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6">
                    Systèmes Électroniques
                    <span className="block bg-gradient-to-r from-blue-600 to-purple-600 bg-clip-text text-transparent">
                      Avancés
                    </span>
                  </h1>
                  
                  <p className="text-xl text-muted-foreground max-w-3xl mx-auto mb-8">
                    Conception et réalisation des systèmes de contrôle électronique 
                    pour le convoyeur de tri automatisé avec capteurs intelligents.
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
                      Objectifs Électroniques
                    </CardTitle>
                    <CardDescription>
                      Systèmes embarqués et capteurs pour l'automatisation industrielle
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <ul className="space-y-3">
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-blue-500 rounded-full mt-2"></div>
                        <span>Concevoir et réaliser des PCB personnalisés pour le contrôle</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-blue-500 rounded-full mt-2"></div>
                        <span>Intégrer des capteurs de couleur GY-33 TCS34725</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-blue-500 rounded-full mt-2"></div>
                        <span>Développer le firmware pour ESP32</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-blue-500 rounded-full mt-2"></div>
                        <span>Implémenter la communication temps réel</span>
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
                      Composants & Technologies
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="grid md:grid-cols-2 gap-6">
                      <div>
                        <h4 className="font-semibold mb-2">Microcontrôleurs</h4>
                        <p className="text-muted-foreground text-sm">ESP32, Arduino</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">Capteurs</h4>
                        <p className="text-muted-foreground text-sm">TCS34725 (couleur), Encodeurs</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">PCB</h4>
                        <p className="text-muted-foreground text-sm">Circuits personnalisés</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">Communication</h4>
                        <p className="text-muted-foreground text-sm">WiFi, UART, I2C</p>
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
                      Consultez la documentation technique complète du projet :
                    </p>
                    <Link href="/docs/test-final/electronique/rapport-electronique">
                      <Button variant="outline" className="w-full justify-start">
                        <Cpu className="w-4 h-4 mr-2" />
                        Rapport Électronique Complet
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
                      Points Clés du Projet
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-3">
                    <div className="text-sm space-y-2">
                      <p>• Intégration complète des systèmes électroniques</p>
                      <p>• Précision dans la détection des couleurs</p>
                      <p>• Robustesse des communications industrielles</p>
                      <p>• Documentation technique professionnelle</p>
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
                  <Link href="/docs/test-final">
                    <Button variant="outline" className="gap-2 group">
                      <ArrowLeft className="w-4 h-4 group-hover:-translate-x-1 transition-transform" />
                      Test Final
                    </Button>
                  </Link>
                  
                  <Link href="/docs/test-final/it">
                    <Button className="gap-2 group">
                      IT Test Final
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
