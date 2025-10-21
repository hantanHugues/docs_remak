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
  Code,
  Target, 
  Clock,
  BookOpen,
  AlertCircle,
  Brain
} from "lucide-react";
import Link from "next/link";

export default function TestFinalITPage() {
  const sections = [
    {
      title: "Classification Couleur",
      description: "Algorithmes intelligents de détection et classification des couleurs",
      href: "/docs/test-final/it/classification-couleur",
      icon: Brain,
      color: "from-green-500 to-emerald-500"
    },
    {
      title: "Dashboard Industriel", 
      description: "Interface de supervision temps réel pour le système de tri",
      href: "/docs/test-final/it/dashboard-industriel",
      icon: Code,
      color: "from-blue-500 to-indigo-500"
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
            <section className="relative py-20 bg-gradient-to-br from-green-50 via-emerald-50 to-teal-50 dark:from-green-900/10 dark:via-emerald-900/10 dark:to-teal-900/10">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto text-center">
                  <Badge variant="outline" className="mb-6 bg-white/50 dark:bg-gray-800/50">
                    <Code className="w-4 h-4 mr-2" />
                    Test Final - Informatique
                  </Badge>
                  
                  <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6">
                    Systèmes Intelligents
                    <span className="block bg-gradient-to-r from-green-600 to-emerald-600 bg-clip-text text-transparent">
                      & Supervision
                    </span>
                  </h1>
                  
                  <p className="text-xl text-muted-foreground max-w-3xl mx-auto mb-8">
                    Développement d'algorithmes de classification intelligente et 
                    d'interfaces de supervision industrielle pour le contrôle temps réel.
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
                  <h2 className="text-3xl font-bold mb-4">Composants Logiciels</h2>
                  <p className="text-muted-foreground max-w-2xl mx-auto">
                    Solutions informatiques avancées pour l'automatisation industrielle 
                    et la supervision temps réel.
                  </p>
                </div>
              </AnimatedSection>

              <div className="grid md:grid-cols-2 gap-8 mb-12">
                {sections.map((section, index) => (
                  <AnimatedSection key={index} animation="fade-up" delay={index * 150}>
                    <Link href={section.href}>
                      <Card className="group hover:shadow-xl transition-all duration-300 border-0 bg-gradient-to-br from-white to-gray-50/50 dark:from-gray-800 dark:to-gray-900/50 overflow-hidden h-full">
                        <CardHeader className="pb-4">
                          <div className={`w-12 h-12 rounded-lg bg-gradient-to-r ${section.color} p-2.5 mb-4 group-hover:scale-110 transition-transform duration-300`}>
                            <section.icon className="w-full h-full text-white" />
                          </div>
                          
                          <CardTitle className="group-hover:text-green-600 transition-colors">
                            {section.title}
                          </CardTitle>
                          
                          <CardDescription className="text-sm">
                            {section.description}
                          </CardDescription>
                        </CardHeader>
                        
                        <CardContent>
                          <div className="flex items-center text-green-600 mt-4 group-hover:translate-x-1 transition-transform">
                            <span className="text-sm font-medium">Explorer</span>
                            <ArrowRight className="w-4 h-4 ml-1" />
                          </div>
                        </CardContent>
                      </Card>
                    </Link>
                  </AnimatedSection>
                ))}
              </div>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Target className="w-5 h-5 text-green-600" />
                      Objectifs Informatiques
                    </CardTitle>
                    <CardDescription>
                      Intelligence artificielle et interfaces utilisateur industrielles
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <ul className="space-y-3">
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-green-500 rounded-full mt-2"></div>
                        <span>Développer des algorithmes de classification couleur précis</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-green-500 rounded-full mt-2"></div>
                        <span>Créer un dashboard industriel temps réel</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-green-500 rounded-full mt-2"></div>
                        <span>Implémenter la correction gamma et l'amélioration des capteurs</span>
                      </li>
                      <li className="flex items-start gap-3">
                        <div className="w-2 h-2 bg-green-500 rounded-full mt-2"></div>
                        <span>Intégrer des perspectives de Machine Learning</span>
                      </li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={200}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Clock className="w-5 h-5 text-blue-600" />
                      Technologies Utilisées
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="grid md:grid-cols-2 gap-6">
                      <div>
                        <h4 className="font-semibold mb-2">Frontend</h4>
                        <p className="text-muted-foreground text-sm">React, Next.js, Streamlit</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">Backend</h4>
                        <p className="text-muted-foreground text-sm">Python, FastAPI, WebSockets</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">Machine Learning</h4>
                        <p className="text-muted-foreground text-sm">Scikit-learn, TensorFlow</p>
                      </div>
                      <div>
                        <h4 className="font-semibold mb-2">Déploiement</h4>
                        <p className="text-muted-foreground text-sm">Vercel, Docker</p>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={300}>
                <Card className="border-yellow-200 bg-yellow-50/50 dark:border-yellow-800 dark:bg-yellow-900/10">
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2 text-yellow-800 dark:text-yellow-200">
                      <AlertCircle className="w-5 h-5" />
                      Innovations Techniques
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-3">
                    <div className="text-sm space-y-2">
                      <p>• Algorithmes de classification adaptative des couleurs</p>
                      <p>• Interface de supervision industrielle moderne</p>
                      <p>• Intégration temps réel avec capteurs IoT</p>
                      <p>• Architecture scalable et maintenant</p>
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
                  <Link href="/docs/test-final/electronique">
                    <Button variant="outline" className="gap-2 group">
                      <ArrowLeft className="w-4 h-4 group-hover:-translate-x-1 transition-transform" />
                      Électronique
                    </Button>
                  </Link>
                  
                  <Link href="/docs/test-final/mecanique">
                    <Button className="gap-2 group">
                      Mécanique
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
