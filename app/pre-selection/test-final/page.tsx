"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Alert, AlertDescription } from "@/components/ui/alert";
import {
  ArrowRight, ArrowLeft, Trophy, AlertTriangle, Calendar, Clock
} from "lucide-react";
import Link from "next/link";

export default function TestFinalPage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* En-tête */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-16 bg-gradient-to-br from-orange-50 via-white to-red-50 dark:from-orange-950/20 dark:via-background dark:to-red-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-6">
                    <Link href="/pre-selection" className="hover:text-foreground transition-colors">
                      Pré-sélection
                    </Link>
                    <span>/</span>
                    <span>Test Final</span>
                  </div>

                  <div className="flex items-start gap-6 mb-8">
                    <div className="w-16 h-16 rounded-2xl bg-gradient-to-r from-orange-600 to-red-600 flex items-center justify-center shadow-lg">
                      <Trophy className="w-8 h-8 text-white" />
                    </div>
                    <div>
                      <Badge className="mb-4 bg-orange-100 text-orange-800 hover:bg-orange-200 dark:bg-orange-900 dark:text-orange-100">
                        Phase Finale
                      </Badge>
                      <h1 className="text-4xl md:text-5xl font-bold mb-4 bg-gradient-to-r from-orange-600 to-red-600 bg-clip-text text-transparent">
                        Test Final
                      </h1>
                      <p className="text-xl text-muted-foreground leading-relaxed">
                        Système de convoyeur intelligent pour le tri de déchets
                      </p>
                    </div>
                  </div>

                  <div className="flex flex-wrap gap-3">
                    <Badge variant="secondary" className="flex items-center gap-1">
                      <Calendar className="w-3 h-3" />
                      À venir
                    </Badge>
                    <Badge variant="outline">Convoyeur</Badge>
                    <Badge variant="outline">Tri intelligent</Badge>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Contenu */}
          <div className="container mx-auto px-4 py-12">
            <div className="max-w-4xl mx-auto space-y-8">
              
              <Alert className="border-orange-200 bg-orange-50 dark:border-orange-800 dark:bg-orange-950/20">
                <AlertTriangle className="h-4 w-4 text-orange-600" />
                <AlertDescription className="text-orange-800 dark:text-orange-200">
                  <strong>Documentation à venir :</strong> La documentation détaillée de cette phase sera publiée au début de la phase finale de la compétition.
                </AlertDescription>
              </Alert>

              <AnimatedSection animation="fade-up">
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2 text-orange-700 dark:text-orange-300">
                      <Clock className="w-5 h-5" />
                      Contenu en préparation
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <p className="text-muted-foreground">
                      Cette section contiendra bientôt :
                    </p>
                    <ul className="space-y-2 text-sm text-muted-foreground">
                      <li>• Instructions détaillées du test final</li>
                      <li>• Critères d'évaluation par domaine</li>
                      <li>• Ressources et documentation technique</li>
                      <li>• Planning et modalités de présentation</li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Navigation */}
              <div className="flex justify-between items-center pt-8 border-t">
                <Link href="/docs/semaine-3">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Semaine 3 - Avancés
                  </Button>
                </Link>
                <Link href="/finale">
                  <Button>
                    Finale
                    <ArrowRight className="w-4 h-4 ml-2" />
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
