"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, HardHat, Cog, BookOpen, Download, PlayCircle, FolderArchive, Target, CheckCircle, BarChart2, AlertTriangle, ExternalLink, FileText, Settings, Layers
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';

export default function MechanicsAdvancedTestPage() {
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
                      Documentation Mécanique
                    </Link>
                    <span>/</span>
                    <span>Test 3 : Niveau Avancé</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <HardHat className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">🛠️ Documentation – Test 3 : Niveau Avancé</h1>
                      <p className="text-muted-foreground">Conception, modélisation et validation d'une pièce mécanique complexe.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Mécanique</Badge>
                    <Badge variant="outline">SolidWorks</Badge>
                    <Badge variant="outline">CAO</Badge>
                    <Badge variant="outline">Avancé</Badge>
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
              
              {/* Sections */}
              <div className="space-y-12">

                <AnimatedSection animation="fade-up">
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <Target className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        1. 🎯 Contexte et Objectif du Test
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6">
                      <p className="font-semibold">Test 3 – Niveau avancé</p>
                      <p className="mt-2 text-gray-700 dark:text-gray-300">Évaluer la capacité à concevoir, modéliser et valider une pièce mécanique complexe en respectant :</p>
                      <ul className="list-disc pl-5 mt-2 space-y-1 text-sm text-gray-600 dark:text-gray-400">
                        <li>la géométrie donnée (plans + rendus 3D)</li>
                        <li>la masse cible (calcul à la décimale près)</li>
                        <li>la gestion des erreurs d’unités et d’arrondis</li>
                      </ul>
                    </CardContent>
                  </Card>
                </AnimatedSection>

                <AnimatedSection animation="fade-up" delay={50}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <Settings className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        2. 📐 Spécifications et livrables
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6">
                      <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                        <li><strong>Unité</strong> : MMGS (millimètre, gramme, seconde)</li>
                        <li><strong>Décimales</strong> : 2</li>
                        <li><strong>Matériau</strong> : Aluminium 1060 (ρ = 2700 kg/m³)</li>
                        <li><strong>Congés filetés</strong> : 12 × R10</li>
                        <li><strong>Trous</strong> : tous débouchants sauf indication contraire</li>
                      </ul>
                      <h3 className="font-semibold mt-4 mb-2">Livrables</h3>
                      <ol className="list-decimal pl-5 space-y-1 text-sm text-gray-600 dark:text-gray-400">
                        <li>Fichier CAO de la pièce modélisée (<code>.SLDPRT</code> ou équivalent)</li>
                        <li>Tableau de calcul des masses pour les trois jeux de dimensions</li>
                        <li>Rapport détaillé décrivant la démarche et les arrondis appliqués</li>
                      </ol>
                    </CardContent>
                  </Card>
                </AnimatedSection>
                
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        3. Processus et méthodologie
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 space-y-4">
                      <p>Nous avons respecté toutes les côtes et tolérances du document fourni. Chaque étape contient un emplacement pour insérer vos captures d’écran afin d’illustrer l’avancement.</p>
                      <h3 className="font-medium">1. Import des plans et préparation du fichier</h3>
                      <h3 className="font-medium">2. Esquisse de la forme principale</h3>
                      <h3 className="font-medium">3. Extrusion et découpe initiale</h3>
                      <h3 className="font-medium">4. Création des évidements secondaires et rainures</h3>
                      <h3 className="font-medium">5. Perçages et congés</h3>
                      <h3 className="font-medium">6. Découpe inclinée et formes angulaires</h3>
                      <h3 className="font-medium">7. Détails finaux et contrôle qualité</h3>
                    </CardContent>
                  </Card>
                </AnimatedSection>

                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        Illustrations — Processus (images)
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6">
                      <div className="grid grid-cols-2 sm:grid-cols-3 lg:grid-cols-4 gap-4">
                        {[...Array(29)].map((_, i) => (
                          <Image key={i} src={`/assets/imgs/img_${i + 1}.png`} alt={`Processus image ${i + 1}`} width={150} height={100} className="rounded-md border"/>
                        ))}
                      </div>
                      <p className="text-sm text-center mt-4 italic text-gray-500">Figure : Processus en image de construction de la pièce du test 3.</p>
                    </CardContent>
                  </Card>
                </AnimatedSection>

                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        4. 🛠️ Tâches à réaliser
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6">
                      <p className="mb-4">Pour chaque jeu de dimensions, calculer la masse de la pièce (en grammes) :</p>
                      <ol className="list-decimal pl-5 space-y-2 mb-4">
                        <li><strong>Q3a.</strong> A = 193 mm ; B = 88 mm ; W = B/2 ; X = A/4 ; Y = B + 5,5 mm ; Z = B + 15 mm</li>
                        <li><strong>Q3b.</strong> A = 205 mm ; B = 100 mm ; W = B/2 ; X = A/4 ; Y = B + 5,5 mm ; Z = B + 15 mm</li>
                        <li><strong>Q3c.</strong> A = 210 mm ; B = 105 mm ; W = B/2 ; X = A/4 ; Y = B + 5,5 mm ; Z = B + 15 mm</li>
                      </ol>
                      <p className="font-semibold">À fournir :</p>
                      <ol className="list-decimal pl-5 text-sm text-gray-600 dark:text-gray-400">
                        <li>Les valeurs numériques (masse en g, arrondie à 2 décimales)</li>
                        <li>Capture d’écran du calcul de volume/masse dans le logiciel CAO</li>
                      </ol>
                    </CardContent>
                  </Card>
                </AnimatedSection>
                
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        5. ✅ Critères de réussite
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6">
                      <ol className="list-decimal pl-5 space-y-2">
                        <li><strong>Exactitude</strong> des masses (&lt; ± 1 % d’écart)</li>
                        <li><strong>Conformité géométrique</strong> (tolérances dimensionnelles respectées)</li>
                        <li><strong>Clarté</strong> du rapport</li>
                        <li><strong>Qualité</strong> du fichier CAO (nommage, structure, mise en plan propre)</li>
                      </ol>
                    </CardContent>
                  </Card>
                </AnimatedSection>
                
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        6. 🧩 Pièce à Modéliser
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6">
                      <div className="grid md:grid-cols-2 gap-4">
                        <figure><Image src="/assets/imgs/a_modeliser_1.png" alt="Rendu 1 - vue de dessus" width={400} height={300} className="rounded-md border"/><figcaption className="text-sm italic text-center mt-2">Rendu 1 : Vue de dessus - de droite - trisométrie</figcaption></figure>
                        <figure><Image src="/assets/imgs/a_modeliser_2.png" alt="Rendu 2 - vue de face" width={400} height={300} className="rounded-md border"/><figcaption className="text-sm italic text-center mt-2">Rendu 2 : Vue de face - trisométrie</figcaption></figure>
                      </div>
                    </CardContent>
                  </Card>
                </AnimatedSection>
                
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        7. 📊 Présentation des Résultats
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 space-y-6">
                      <div className="overflow-x-auto">
                        <table className="w-full text-sm">
                          <thead><tr className="bg-gray-50 dark:bg-gray-800"><th className="px-4 py-2 border">Cas</th><th className="px-4 py-2 border">A (mm)</th><th className="px-4 py-2 border">B (mm)</th><th className="px-4 py-2 border">Masse calculée (g)</th></tr></thead>
                          <tbody>
                            <tr><td className="px-4 py-2 border">Q3a</td><td className="px-4 py-2 border">193</td><td className="px-4 py-2 border">88</td><td className="px-4 py-2 border"><strong>1393,82</strong></td></tr>
                            <tr><td className="px-4 py-2 border">Q3b</td><td className="px-4 py-2 border">205</td><td className="px-4 py-2 border">100</td><td className="px-4 py-2 border"><strong>1492,49</strong></td></tr>
                            <tr><td className="px-4 py-2 border">Q3c</td><td className="px-4 py-2 border">210</td><td className="px-4 py-2 border">105</td><td className="px-4 py-2 border"><strong>1531,19</strong></td></tr>
                          </tbody>
                        </table>
                      </div>
                      <h3 className="font-medium">Captures d’écran des masses obtenues</h3>
                      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                        <figure><Image src="/assets/imgs/a_masse.png" alt="Capture volume Q3a" width={200} height={150} className="rounded-md border"/><figcaption className="text-xs italic text-center mt-1">Figure 1 : Calcul pour Q3a</figcaption></figure>
                        <figure><Image src="/assets/imgs/b_masse.png" alt="Capture volume Q3b" width={200} height={150} className="rounded-md border"/><figcaption className="text-xs italic text-center mt-1">Figure 2 : Calcul pour Q3b</figcaption></figure>
                        <figure><Image src="/assets/imgs/c_masse.png" alt="Capture volume Q3c" width={200} height={150} className="rounded-md border"/><figcaption className="text-xs italic text-center mt-1">Figure 3 : Calcul pour Q3c</figcaption></figure>
                      </div>
                      <h3 className="font-medium">Pièces réalisées</h3>
                      <ol className="list-decimal pl-5 text-sm text-blue-600 dark:text-blue-400">
                        <li><Link href="/assets/pieces/third_test-final_piece-a.SLDPRT" className="hover:underline">Pièce finale - Q3a</Link></li>
                        <li><Link href="/assets/pieces/third_test_final_piece-b.SLDPRT" className="hover:underline">Pièce finale - Q3b</Link></li>
                        <li><Link href="/assets/pieces/third_test_final_piece-c.SLDPRT" className="hover:underline">Pièce finale - Q3c</Link></li>
                      </ol>
                      <h3 className="font-medium">Vidéos illustratives</h3>
                      <div className="aspect-video rounded-lg overflow-hidden border">
                        <iframe src="https://player.vimeo.com/video/1097167515?h=204e839d21" width="100%" height="100%" frameBorder="0" allow="autoplay; fullscreen; picture-in-picture" allowFullScreen></iframe>
                      </div>
                    </CardContent>
                  </Card>
              </AnimatedSection>
                
              </div>

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