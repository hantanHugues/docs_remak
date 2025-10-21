"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Alert, AlertDescription } from "@/components/ui/alert";
import {
  ArrowLeft, BookOpen, AlertTriangle, Clock
} from "lucide-react";
import Link from "next/link";

export default function RessourcesPage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* En-tête */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-12 bg-white dark:bg-gray-900 border-b border-gray-200 dark:border-gray-700">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-4">
                    <Link href="/pre-selection" className="hover:text-foreground transition-colors">
                      Pré-sélection
                    </Link>
                    <span>/</span>
                    <Link href="/pre-selection/test-final" className="hover:text-foreground transition-colors">
                      Test Final
                    </Link>
                    <span>/</span>
                    <span>Ressources</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <BookOpen className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Ressources</h1>
                      <p className="text-muted-foreground">Documentation technique et outils pour le test final</p>
                    </div>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Contenu */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-4xl mx-auto space-y-8">
              
              <Alert className="border-orange-200 bg-orange-50 dark:border-orange-800 dark:bg-orange-950/20">
                <AlertTriangle className="h-4 w-4 text-orange-600" />
                <AlertDescription className="text-orange-800 dark:text-orange-200">
                  <strong>Documentation à venir :</strong> Les ressources techniques seront publiées au début de la phase finale.
                </AlertDescription>
              </Alert>

              <AnimatedSection animation="fade-up">
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Clock className="w-5 h-5 text-orange-600" />
                      Contenu en préparation
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <p className="text-muted-foreground mb-4">
                      Cette page contiendra prochainement :
                    </p>
                    <ul className="space-y-2 text-sm text-muted-foreground">
                      <li>• Documentation technique détaillée</li>
                      <li>• Schémas et plans de référence</li>
                      <li>• Codes d'exemple et bibliothèques</li>
                      <li>• Tutoriels et guides pratiques</li>
                      <li>• Ressources pour la préparation</li>
                      <li>• FAQ et conseils techniques</li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Navigation */}
              <div className="flex justify-between items-center pt-8 border-t">
                <Link href="/pre-selection/test-final/criteres">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Critères d'évaluation
                  </Button>
                </Link>
                <Link href="/finale">
                  <Button>
                    Finale
                  </Button>
                </Link>
              </div>
            </div>
          </div>
        </main>
      </div>
    </div>
  )
}
