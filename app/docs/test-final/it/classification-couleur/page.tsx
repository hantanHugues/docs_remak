"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Code, BookOpen, Settings, List, FileText, Cpu, AlertTriangle, Download, CheckCircle, Target, HardHat, GitBranch, PlayCircle, Lightbulb, Bot
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';

export default function ColorDetectionDocPage() {
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
                      Documentation Informatique
                    </Link>
                    <span>/</span>
                    <span>Détection Intelligente des Couleurs</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Lightbulb className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Documentation : Détection Intelligente des Couleurs pour le Système de Convoyeur</h1>
                      <p className="text-muted-foreground">Processus de développement et approches explorées pour l'identification de couleurs.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Informatique</Badge>
                    <Badge variant="outline">Capteurs</Badge>
                    <Badge variant="outline">Arduino</Badge>
                    <Badge variant="outline">Vision</Badge>
                   </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
            <div className="container mx-auto px-4">
              <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
                <Link href="#">
                  <Button variant="ghost" size="sm">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Page Précédente
                  </Button>
                </Link>
                <Link href="#">
                  <Button variant="ghost" size="sm">
                    Page Suivante
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
                      Table des Matières
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ol className="list-decimal pl-5 space-y-2 text-blue-600 dark:text-blue-400">
                      <li><a href="#introduction" className="hover:underline">Introduction</a></li>
                      <li><a href="#composants-utilisés" className="hover:underline">Composants Utilisés</a></li>
                      <li><a href="#approches-de-détection-des-couleurs" className="hover:underline">Approches de Détection des Couleurs</a></li>
                      <li><a href="#défis-rencontrés" className="hover:underline">Défis Rencontrés</a></li>
                      <li><a href="#algorithme-de-gestion-du-tri" className="hover:underline">Algorithme de Gestion du Tri</a></li>
                      <li><a href="#conclusion" className="hover:underline">Conclusion</a></li>
                      <li><a href="#références" className="hover:underline">Références</a></li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={50}>
                <Card id="introduction" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Introduction
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 text-gray-700 dark:text-gray-300 leading-relaxed space-y-4">
                    <p>Notre équipe IT a développé un système de détection intelligente des couleurs pour un convoyeur destiné au tri de déchets. L'objectif principal est d'identifier quatre types de déchets en fonction de leur couleur : rouge, bleu, vert et jaune. Ce système utilise le capteur de couleur GY-33 TCS34725, intégré au convoyeur, pour capturer les valeurs RVB (Rouge, Vert, Bleu) des objets qui passent devant lui.</p>
                    <p>Cette documentation présente notre processus de développement, les composants employés, les différentes approches explorées pour optimiser la détection, ainsi que les défis rencontrés. Nous expliquerons également des concepts techniques clés, comme la correction gamma, et fournirons des extraits de code pour illustrer nos solutions.</p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="composants-utilisés" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Cpu className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Composants Utilisés
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">Capteur de Couleur GY-33 TCS34725</h3>
                    <p className="text-gray-700 dark:text-gray-300">Le <strong>GY-33 TCS34725</strong> est un capteur de couleur numérique qui utilise une interface I2C pour communiquer avec un microcontrôleur.</p>
                    <h4 className="font-medium text-gray-800 dark:text-gray-200">Caractéristiques Principales :</h4>
                    <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400">
                      <li><strong>Interface I2C</strong>, <strong>Temps d'intégration ajustable</strong>, <strong>Gain ajustable</strong>.</li>
                      <li><strong>Sorties</strong> : Valeurs brutes pour les canaux rouge (R), vert (G), bleu (B) et clair (C).</li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="approches-de-détection-des-couleurs" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <GitBranch className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Approches de Détection des Couleurs
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`uint16_t clear, red, green, blue;
tcs.getRawData(&red, &green, &blue, &clear);
float r = (float)red / clear * 255.0;
// ...`}</code></pre></div>
                    <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">Approche 1 : Classification Basique</h3>
                    <p>Classification simple via des seuils RVB. Précise de près (&lt; 5 cm), mais limitée à distance.</p>
                    <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">Approche 2 : Amélioration avec Correction Gamma</h3>
                    <p>Introduction d'une **correction gamma** (exposant 2.5) pour compenser la non-linéarité du capteur et stabiliser les lectures face aux variations de luminosité et de distance.</p>
                    <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">Approche 3 : Détection de Couleur Améliorée pour 10-12 cm</h3>
                    <p>Optimisation des paramètres du capteur et moyennage de plusieurs lectures pour réduire le bruit, améliorant significativement la précision à 10-12 cm.</p>
                    <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">Approche 4 : Calibration du Convoyeur</h3>
                    <p>Procédure de calibration interactive pour ajuster la balance des blancs et apprendre les teintes de référence (espace HSV), améliorant la robustesse.</p>
                    <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">Approche 5 : Perspectives avec Machine Learning</h3>
                    <p>Envisager un classificateur (ex: KNN) entraîné sur un jeu de données pour une prédiction plus précise et adaptable. Non implémenté par manque de temps.</p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="défis-rencontrés" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <AlertTriangle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Défis Rencontrés
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ol className="list-decimal pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li><strong>Distance Capteur-Objet</strong> : Résolu par correction gamma et calibration.</li>
                      <li><strong>Conditions d'Éclairage</strong> : Résolu par normalisation et balance des blancs.</li>
                      <li><strong>Classification des Couleurs</strong> : Résolu par itérations et ajustements via calibration.</li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="algorithme-de-gestion-du-tri" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Bot className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Algorithme de Gestion du Tri
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ol className="list-decimal pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li><strong>Détection</strong> : Le capteur GY-33 TCS34725 capture les valeurs RVB et C.</li>
                      <li><strong>Identification</strong> : Les valeurs sont normalisées, corrigées, puis classifiées.</li>
                      <li><strong>Consigne de Tri</strong> : Selon la couleur, une consigne est envoyée au système mécanique (prévu mais non implémenté dans ce code).</li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="conclusion" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Conclusion
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300">
                      Notre système de détection, basé sur le capteur GY-33 TCS34725, permet d'identifier avec précision les couleurs à des distances de 10-12 cm. Grâce à des approches comme la correction gamma et la calibration, nous avons surmonté les défis liés à la distance et à l'éclairage. Des améliorations futures, comme un capteur de présence ou un modèle de machine learning, pourraient encore optimiser le système.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="références" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      Références
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ul className="list-disc pl-5 space-y-2 text-blue-600 dark:text-blue-400">
                      <li><Link href="https://youpilab.com/components/product/capteur-de-couleur-gy-33-tcs34725" className="hover:underline">Documentation du GY-33 TCS34725</Link></li>
                      <li><Link href="https://fr.wikipedia.org/wiki/Correction_gamma" className="hover:underline">Correction Gamma - Wikipédia</Link></li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 border-t">
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