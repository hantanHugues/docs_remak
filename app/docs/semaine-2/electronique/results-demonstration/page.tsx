"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { PageNavigation } from "@/components/page-navigation";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, CheckCircle, Database, GitCompareArrows, Monitor, Activity, ExternalLink
} from "lucide-react";
import Link from "next/link";
import { useMounted } from "@/hooks/use-mounted";


export default function ResultsDemoFRPage() {
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
          {/* En-tête */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-12 bg-gradient-to-br from-green-50 via-white to-lime-50 dark:from-green-950/20 dark:via-background dark:to-lime-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-4">
                    <Link href="#" className="hover:text-foreground transition-colors">
                      Électronique
                    </Link>
                    <span>/</span>
                    <span>Résultats et Démonstration</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <CheckCircle className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Résultats et Démonstration - Boîte Noire & Station de Contrôle</h1>
                      <p className="text-muted-foreground">Validation des fonctionnalités clés, de l'acquisition des données à la détection de crash.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Validation</Badge>
                    <Badge variant="outline">Tests</Badge>
                    <Badge variant="outline">Résultats</Badge>
                    <Badge variant="outline">Électronique</Badge>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <PageNavigation />

          {/* Contenu */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-5xl mx-auto">
              
              <AnimatedSection animation="fade-up">
                <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-8 mb-12">
                  <p className="text-center text-gray-700 dark:text-gray-300 leading-relaxed">
                    Ce projet a été testé de manière rigoureuse pour valider ses fonctionnalités clés, de l'acquisition de données à la détection de crash et à la récupération des informations.
                  </p>
                </div>
              </AnimatedSection>
              
              <Separator className="mb-12" />

              {/* SECTION 1 - Tests Fonctionnels */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-green-500 mt-12">
                  <CardHeader className="bg-gradient-to-r from-green-50 to-transparent dark:from-green-950/20">
                    <CardTitle id="tests-fonctionnels" className="flex items-center gap-3 scroll-mt-24 text-xl">
                      <CheckCircle className="w-6 h-6 text-green-600" />
                      1. Tests Fonctionnels
                    </CardTitle>
                    <CardDescription>
                      Des tests unitaires et d'intégration ont été réalisés pour chaque composant et module. Les résultats détaillés de ces tests sont disponibles dans le dossier <code>tests/results/</code>.
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-6 pt-6">
                    
                    {/* Test EEPROM */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
                        <div className="flex items-center gap-4">
                          <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                            <Database className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                          </div>
                          <div>
                            <h4 className="font-semibold text-gray-900 dark:text-gray-100">Test de l'EEPROM (<code>eeprom_test</code>)</h4>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Validation de la lecture et de l'écriture de données non volatiles.
                            </p>
                          </div>
                        </div>
                        <a href="/Documentation/semaine-2/electronique/tests/results/eeprom_test/results.txt" target="_blank" rel="noopener noreferrer">
                          <Button variant="outline" size="sm">
                            Voir les résultats
                            <ExternalLink className="w-3 h-3 ml-2" />
                          </Button>
                        </a>
                      </div>
                    </div>

                    {/* Test Communication I2C */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
                        <div className="flex items-center gap-4">
                          <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                            <GitCompareArrows className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                          </div>
                          <div>
                            <h4 className="font-semibold text-gray-900 dark:text-gray-100">Test de Communication I2C (<code>i2c_master_to_slave</code>)</h4>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Vérification de la transmission bidirectionnelle entre la Boîte Noire (Maître) et la Station de Contrôle (Esclave).
                            </p>
                          </div>
                        </div>
                        <a href="/Documentation/semaine-2/electronique/tests/results/i2c_master_to_slave/results.txt" target="_blank" rel="noopener noreferrer">
                          <Button variant="outline" size="sm">
                            Voir les résultats
                            <ExternalLink className="w-3 h-3 ml-2" />
                          </Button>
                        </a>
                      </div>
                    </div>

                    {/* Test Écran LCD I2C */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
                        <div className="flex items-center gap-4">
                          <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                            <Monitor className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                          </div>
                          <div>
                            <h4 className="font-semibold text-gray-900 dark:text-gray-100">Test de l'Écran LCD I2C (<code>lcd_i2c_test</code>)</h4>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Confirmation de l'affichage correct des informations.
                            </p>
                          </div>
                        </div>
                        <a href="/Documentation/semaine-2/electronique/tests/results/lcd_i2c_test/results.txt" target="_blank" rel="noopener noreferrer">
                          <Button variant="outline" size="sm">
                            Voir les résultats
                            <ExternalLink className="w-3 h-3 ml-2" />
                          </Button>
                        </a>
                      </div>
                    </div>

                    {/* Test Capteur MPU-6050 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
                        <div className="flex items-center gap-4">
                          <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                            <Activity className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                          </div>
                          <div>
                            <h4 className="font-semibold text-gray-900 dark:text-gray-100">Test du Capteur MPU-6050 (<code>mpu6050_test</code>)</h4>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Vérification de l'acquisition précise des données d'accélération et de gyroscope.
                            </p>
                          </div>
                        </div>
                        <a href="/Documentation/semaine-2/electronique/tests/results/mpu6050_test/results.txt" target="_blank" rel="noopener noreferrer">
                          <Button variant="outline" size="sm">
                            Voir les résultats
                            <ExternalLink className="w-3 h-3 ml-2" />
                          </Button>
                        </a>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 mt-12 border-t">
                <Link href="#">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Page Précédente
                  </Button>
                </Link>
                <Link href="#">
                  <Button>
                    Page Suivante
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