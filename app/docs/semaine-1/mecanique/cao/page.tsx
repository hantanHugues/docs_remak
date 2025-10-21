"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import {
  ArrowLeft, ArrowRight, HardHat, Cog, BookOpen, Download, PlayCircle, FolderArchive
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';

export default function MechanicsTestsPage() {
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
                      TRC 2025
                    </Link>
                    <span>/</span>
                    <span>Tests de Mécanique</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <HardHat className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Tests de mécaniques - TRC 2025</h1>
                      <p className="text-muted-foreground">Conception et modélisation de pièces sous contraintes.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Mécanique</Badge>
                    <Badge variant="outline">SolidWorks</Badge>
                    <Badge variant="outline">CAO</Badge>
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
                      <Cog className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Test 1 - Niveau débutant
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300">
                      Ce test consiste à concevoir un certain nombre de pièces en respectant les contraintes imposées (matériaux, dimensions ...). 
                      Pour ce faire, il faudra réaliser un croquis incluant des formes géométriques de base telles que des rectangles, des cercles et des polygones 
                      pour enfin arriver à modéliser des pièces tridimensionnelles à partir des croquis créés.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={50}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Technologie
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300">
                      La conception des pièces a été réalisée avec le logiciel de modélisation <strong className="font-semibold text-primary">SolidWorks 2025</strong>.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      Présentation des croquis
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    
                    {/* Pièce 1 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-4 text-lg">Pièce 1</h3>
                      <div className="space-y-4">
                        <div>
                          <h4 className="font-semibold">Caractéristiques :</h4>
                          <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400">
                            <li><strong>Systèmes d'unités</strong> : MMGS (millimètre, gramme, seconde)</li>
                            <li><strong>Décimales</strong> : 2</li>
                            <li>Tous les trous sont débouchants sauf indication contraire</li>
                            <li><strong>Matériau</strong> : acier AISI 1020 ; Densité : 0,0079 g/mm³</li>
                          </ul>
                        </div>
                        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                          <Image src="/assets/imgs/picture_piece_1_full.jpg" alt="Pièce 1 - Vue détaillée" width={300} height={200} className="rounded-md border"/>
                          <Image src="/assets/imgs/picture_piece_1_full.jpg" alt="Pièce 1 - Vue complète" width={300} height={200} className="rounded-md border"/>
                        </div>
                        <div>
                          <h4 className="font-semibold">Étapes de réalisation</h4>
                          <ol className="list-decimal pl-5 text-sm text-gray-600 dark:text-gray-400">
                            <li>Création d'un nouveau projet...</li>
                            <li>Extrusion du profil : profondeur 20 mm.</li>
                            <li>Création du trou central : cercle Ø100 mm, coupe sur 20 mm.</li>
                            <li>Création de la gorge annulaire : cercle Ø135.6 / Ø100 mm, extrusion 10 mm.</li>
                            <li>Perçage des petits trous latéraux : Ø50 mm, traversants.</li>
                            <li>Configurer matière : acier AISI 1020.</li>
                            <li>Calcul de masse : 2811.1991 g.</li>
                          </ol>
                        </div>
                        <Button asChild variant="outline" size="sm">
                          <Link href="/assets/pieces/piece_1/piece_1.SLDPRT"><Download className="w-4 h-4 mr-2" />Télécharger Pièce 1</Link>
                        </Button>
                      </div>
                    </div>

                    {/* Pièce 2 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-4 text-lg">Pièce 2</h3>
                      <div className="space-y-4">
                        <div>
                          <h4 className="font-semibold">Caractéristiques :</h4>
                          <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400">
                            <li><strong>Systèmes d'unités</strong> : MMGS</li>
                            <li><strong>Décimales</strong> : 2</li>
                            <li><strong>Matériau</strong> : Aluminium Alliage 1060 ; Densité : 0.0027 g/mm³</li>
                          </ul>
                        </div>
                        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                          <Image src="/assets/imgs/picture_piece_2_trc.jpg" alt="Pièce 2 - Vue détaillée" width={300} height={200} className="rounded-md border"/>
                          <Image src="/assets/imgs/picture_piece_2_full.jpg" alt="Pièce 2 - Vue complète" width={300} height={200} className="rounded-md border"/>
                        </div>
                        <div>
                          <h4 className="font-semibold">Étapes de réalisation</h4>
                          <ol className="list-decimal pl-5 text-sm text-gray-600 dark:text-gray-400">
                            <li>Création d'un nouveau projet</li>
                            <li>Esquisse du profil principal.</li>
                            <li>Fonction Révolution.</li>
                            <li>Ajout des congés latéraux.</li>
                            <li>Création des perçages.</li>
                            <li>Calcul de la masse.</li>
                          </ol>
                        </div>
                        <p className="text-sm">Masse : 290.80 g</p>
                        <Button asChild variant="outline" size="sm">
                          <Link href="/assets/pieces/piece_2/piece_2.SLDPRT"><Download className="w-4 h-4 mr-2" />Télécharger Pièce 2</Link>
                        </Button>
                      </div>
                    </div>

                    {/* Pièce 3 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-4 text-lg">Pièce 3</h3>
                      <div className="space-y-4">
                        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                          <Image src="/assets/imgs/picture_piece_3_trc.jpg" alt="Pièce 3 - Vue détaillée" width={300} height={200} className="rounded-md border"/>
                          <Image src="/assets/imgs/picture_piece_3_full.jpg" alt="Pièce 3 - Vue complète" width={300} height={200} className="rounded-md border"/>
                        </div>
                        <p className="text-sm">Masse : 1633.25 g</p>
                        <Button asChild variant="outline" size="sm">
                          <Link href="/assets/pieces/piece_3/piece_3.SLDPRT"><Download className="w-4 h-4 mr-2" />Télécharger Pièce 3</Link>
                        </Button>
                      </div>
                    </div>
                    
                    {/* Pièce 4 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-4 text-lg">Pièce 4</h3>
                      <div className="space-y-4">
                         <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                          <Image src="/assets/imgs/picture_piece_4_trc.jpg" alt="Pièce 4 - Vue détaillée" width={300} height={200} className="rounded-md border"/>
                          <Image src="/assets/imgs/picture_piece_4_full.jpg" alt="Pièce 4 - Vue complète" width={300} height={200} className="rounded-md border"/>
                        </div>
                        <p className="text-sm">Masse : 297.29 g</p>
                        <Button asChild variant="outline" size="sm">
                          <Link href="/assets/pieces/piece_4/piece_4.SLDPRT"><Download className="w-4 h-4 mr-2" />Télécharger Pièce 4</Link>
                        </Button>
                      </div>
                    </div>

                    {/* Assemblage */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-4 text-lg">Assemblage</h3>
                      <div className="space-y-4">
                         <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                          <Image src="/assets/imgs/Piece_assemblage_trc.jpg" alt="Détails de l'assemblage" width={300} height={200} className="rounded-md border"/>
                          <Image src="/assets/imgs/Piece_assemblage.jpg" alt="Vue d'ensemble de l'assemblage" width={300} height={200} className="rounded-md border"/>
                        </div>
                        <p className="text-sm">L’assemblage forme une pince mécanique respectant les contraintes des plans.</p>
                        <div className="text-sm bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3">
                          <p>Position minimale — Centre de masse : X = <strong>-29.15 mm</strong>, Y = <strong>0.16 mm</strong>, Z = <strong>19.86 mm</strong></p>
                          <p>Position maximale — Centre de masse : X = <strong>-25.78 mm</strong>, Y = <strong>0.06 mm</strong>, Z = <strong>19.86 mm</strong></p>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <FolderArchive className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Emplacement des fichiers
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ul className="list-disc pl-5 space-y-2 text-blue-600 dark:text-blue-400">
                      <li><Link href="/assets/pieces/piece_1/piece_1.SLDPRT" className="hover:underline">Pièce I</Link></li>
                      <li><Link href="/assets/pieces/piece_2/piece_2.SLDPRT" className="hover:underline">Pièce II</Link></li>
                      <li><Link href="/assets/pieces/piece_3/piece_3.SLDPRT" className="hover:underline">Pièce III</Link></li>
                      <li><Link href="/assets/pieces/piece_4/piece_4.SLDPRT" className="hover:underline">Pièce IV</Link></li>
                      <li><Link href="/assets/zips/Dossier_A.zip" className="hover:underline">Dossier A</Link></li>
                      <li><Link href="/assets/zips/Dossier_B.zip" className="hover:underline">Dossier B</Link></li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Vidéo de la Pince en mouvement
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="aspect-video rounded-lg overflow-hidden border border-gray-200 dark:border-gray-700">
                      <iframe 
                        src="https://player.vimeo.com/video/1092855533?h=480fde870c" 
                        width="100%" 
                        height="100%" 
                        frameBorder="0" 
                        allow="autoplay; fullscreen; picture-in-picture" 
                        allowFullScreen>
                      </iframe>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      Contribution au projet
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300">
                      Les apports de contribution sont les bienvenus.
                    </p>
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