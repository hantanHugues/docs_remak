"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Code, BookOpen, Settings, List, FileText, Cpu, AlertTriangle, Download, CheckCircle, Target, HardHat, GitBranch, PlayCircle, Lightbulb
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';

export default function ServoDisplayCodeDocPage() {
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
                    <Link href="#" className="hover:text-foreground transition-colors">
                      Documentation Électronique
                    </Link>
                    <span>/</span>
                    <span>Code de l'Afficheur 7 Segments</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Code className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Documentation - Code de l'Afficheur 7 Segments à Servomoteurs</h1>
                      <p className="text-muted-foreground">Présentation du code pour un afficheur mécanique innovant.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Arduino</Badge>
                    <Badge variant="outline">Firmware</Badge>
                    <Badge variant="outline">Servomoteurs</Badge>
                    <Badge variant="outline">PCA9685</Badge>
                   </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
            <div className="container mx-auto px-4">
              <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
                <Link href="/docs/semaine-3/electronique/afficheur-7-servo-segments">
                  <Button variant="ghost" size="sm">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Matériel Afficheur 7 Segments
                  </Button>
                </Link>
                <Link href="/docs/semaine-3/electronique">
                  <Button variant="ghost" size="sm">
                    Électronique Semaine 3
                    <ArrowRight className="w-4 h-4 ml-2" />
                  </Button>
                </Link>
              </div>
            </div>
          </div>

          {/* Contenu */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-4xl mx-auto space-y-8">
              
              <AnimatedSection animation="fade-up">
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <List className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Table des matières
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ol className="list-decimal pl-5 space-y-2 text-blue-600 dark:text-blue-400">
                      <li><a href="#aperçu-du-projet" className="hover:underline">Aperçu du projet</a></li>
                      <li><a href="#matériel-utilisé" className="hover:underline">Matériel utilisé</a></li>
                      <li><a href="#fonctionnement-général" className="hover:underline">Fonctionnement général</a></li>
                      <li><a href="#processus-de-calibration" className="hover:underline">Processus de calibration</a></li>
                      <li><a href="#code-principal" className="hover:underline">Code principal</a></li>
                      <li><a href="#défis-et-solutions" className="hover:underline">Défis et solutions</a></li>
                      <li><a href="#téléchargement" className="hover:underline">Téléchargement</a></li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={50}>
                <Card id="aperçu-du-projet" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Target className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Aperçu du projet
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed space-y-4">
                      <p>Notre afficheur 7 segments mécanique utilise un microcontrôleur ATmega328P (Arduino) et un contrôleur de servomoteurs PCA9685 pour piloter 7 servomoteurs. Chaque servomoteur déplace physiquement un segment, permettant d'afficher les chiffres de 0 à 9 de manière dynamique.</p>
                      <p>Le système est programmé pour effectuer un comptage alternativement croissant et décroissant de 0 à 9 puis de 9 à 0, avec un intervalle d'une seconde entre chaque changement.</p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="matériel-utilisé" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Cpu className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Matériel utilisé
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li>Microcontrôleur ATmega328P (Arduino)</li>
                      <li>Contrôleur PCA9685 (16 canaux PWM, I²C)</li>
                      <li>7 micro-servomoteurs SG90</li>
                      <li>Segments mécaniques fabriqués sur mesure</li>
                      <li>Alimentation externe pour les servomoteurs</li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="fonctionnement-général" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Fonctionnement général
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>L'afficheur fonctionne selon le principe suivant :</p>
                      <ol className="list-decimal pl-5 mt-4 space-y-2">
                        <li>Chaque segment (a-g) est contrôlé par un servomoteur dédié</li>
                        <li>Les servomoteurs ont deux positions :
                          <ul className="list-['-_'] pl-5 mt-1">
                            <li><strong>Position étendue</strong> : le segment est visible (ON)</li>
                            <li><strong>Position rétractée</strong> : le segment est caché (OFF)</li>
                          </ul>
                        </li>
                        <li>Pour afficher un chiffre, le système positionne chaque segment selon un motif prédéfini</li>
                        <li>Le système change automatiquement de chiffre à intervalle régulier (1 seconde)</li>
                        <li>Une fois atteint 9, il compte à rebours jusqu'à 0, puis recommence</li>
                      </ol>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="processus-de-calibration" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Settings className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Processus de calibration
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300 mb-4">
                      Avant d'utiliser l'afficheur, une calibration précise des servomoteurs est nécessaire. Un programme dédié permet d'ajuster les positions étendues et rétractées pour chaque segment. Cette étape est cruciale car chaque servomoteur peut avoir des caractéristiques légèrement différentes, et les contraintes mécaniques de l'assemblage peuvent varier d'un segment à l'autre.
                    </p>
                    <h3 className="font-semibold text-gray-800 dark:text-gray-200 mb-2">Code de calibration</h3>
                    <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ... (code de calibration complet) ...

void loop() {
  // Boucle vide
}`}</code></pre></div>
                    <h3 className="font-semibold text-gray-800 dark:text-gray-200 mb-2">Instructions de calibration</h3>
                    <ol className="list-decimal pl-5 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                      <li><strong>Sélection du segment</strong>: Modifier la variable <code>TARGET_SEGMENT_PIN</code>.</li>
                      <li><strong>Test initial</strong>: Définir <code>CURRENT_CALIBRATION_PULSE</code> (ex: 300).</li>
                      <li><strong>Téléversement</strong>: Envoyer le code pour observer la position.</li>
                      <li><strong>Ajustement progressif</strong>: Modifier la valeur pour étendre (300-500) ou rétracter (100-300).</li>
                      <li><strong>Répétition</strong>: Téléverser à nouveau jusqu'à la position parfaite.</li>
                      <li><strong>Documentation</strong>: Noter les valeurs dans <code>CAL_PULSE_EXTENDED</code> et <code>CAL_PULSE_RETRACTED</code>.</li>
                      <li><strong>Transfert</strong>: Copier les valeurs finales dans le code principal.</li>
                    </ol>
                    <p className="mt-4 text-sm bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3">
                      Cette procédure, répétée pour les 7 segments, nécessite patience et précision pour assurer un mouvement fluide et fiable.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="code-principal" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <FileText className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Code principal
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300 mb-4">
                      Une fois la calibration effectuée, le code principal permet l'affichage dynamique des chiffres.
                    </p>
                    <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ... (code principal complet) ...

void loop() {
  // ... (logique non bloquante avec millis())
}`}</code></pre></div>
                    <h3 className="font-semibold text-gray-800 dark:text-gray-200 mb-2 mt-6">Points clés du code principal</h3>
                    <ul className="list-disc pl-5 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                      <li><strong>Configuration matérielle</strong>: Initialisation du PCA9685.</li>
                      <li><strong>Définition des motifs</strong>: Tableau <code>digitPatterns</code> pour les chiffres 0-9.</li>
                      <li><strong>Gestion du temps</strong>: `millis()` pour un timing non bloquant.</li>
                      <li><strong>Logique de comptage</strong>: Comptage ascendant/descendant.</li>
                      <li><strong>Contrôle des servomoteurs</strong>: Fonctions `setSegmentState()` et `displayDigit()`.</li>
                    </ul>
                    <h3 className="font-semibold text-gray-800 dark:text-gray-200 mb-2 mt-6">Analyse détaillée des fonctions principales</h3>
                    <div className="space-y-4">
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100"><code>void setSegmentState(...)</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mt-2">Positionne un servomoteur en état "étendu" (1) ou "rétracté" (0) en utilisant les valeurs calibrées.</p>
                      </div>
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100"><code>void displayDigit(...)</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mt-2">Transforme un chiffre en configuration de segments via `digitPatterns` et `setSegmentState`.</p>
                      </div>
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100">Boucle principale <code>loop()</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mt-2">Implémente une logique non bloquante avec `millis()` pour gérer le changement automatique des chiffres.</p>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="défis-et-solutions" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <AlertTriangle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Défis et solutions
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300 mb-4">Le développement de cet afficheur a présenté plusieurs défis techniques que nous avons dû résoudre:</p>
                    <ol className="list-decimal pl-5 space-y-4 text-sm text-gray-600 dark:text-gray-400">
                      <li><strong>Calibration précise des servomoteurs</strong> : Résolu par un programme de calibration dédié.</li>
                      <li><strong>Synchronisation des mouvements</strong> : Résolu en utilisant le PCA9685 pour une meilleure précision temporelle.</li>
                      <li><strong>Programmation non bloquante</strong> : Résolu par l'implémentation d'un système basé sur `millis()`.</li>
                      <li><strong>Contraintes mécaniques et fiabilité</strong> : Atténué par une calibration soigneuse, bien que les liens mécaniques restent un point à améliorer.</li>
                      <li><strong>Alimentation électrique</strong> : Résolu par l'utilisation d'une alimentation externe dédiée (>2A) pour les servomoteurs.</li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="téléchargement" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Download className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Téléchargement
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="flex flex-col md:flex-row gap-4">
                      <Button asChild className="w-full md:w-auto">
                        <Link href="Documentation/semaine-3/electronique/images_vids/calibration_code.ino">
                          <Download className="w-4 h-4 mr-2" />
                          Télécharger le code de calibration
                        </Link>
                      </Button>
                      <Button asChild className="w-full md:w-auto">
                        <Link href="Documentation/semaine-3/electronique/images_vids/main_code.ino">
                          <Download className="w-4 h-4 mr-2" />
                          Télécharger le code principal
                        </Link>
                      </Button>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Conclusion
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      Le projet d'afficheur 7 segments à servomoteurs a été une réussite, même si certains obstacles techniques nous ont empêchés d'achever totalement le prototype. L'absence de résistance pour le reset sur le veroboard, ainsi que la faible résistance mécanique des fils de fer reliant les moteurs aux segments, ont constitué des freins majeurs à la finalisation complète du système. Malgré ces défis — utilisation d'un ATmega328P nu, contraintes de temps, et nécessité d'une calibration mécanique précise — nous avons su concevoir et réaliser un prototype fonctionnel et innovant. L'approche non bloquante du code et la rigueur apportée à la calibration ont été des facteurs clés de réussite. Ce projet illustre notre capacité à transformer une idée originale en réalisation concrète, en relevant des défis matériels et logiciels complexes.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-3/electronique/afficheur-7-servo-segments">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Matériel Afficheur 7 Segments
                  </Button>
                </Link>
                <Link href="/docs/semaine-3/electronique">
                  <Button>
                    Électronique Semaine 3
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