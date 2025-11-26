"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { PageNavigation } from "@/components/page-navigation";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import {
  ArrowLeft, ArrowRight, HardHat, Cog, BookOpen, Download, PlayCircle, FolderArchive, Settings, Layers, Play
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
          <PageNavigation />

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
                    
                    {/* Pièces mécaniques */}
                    <AnimatedSection animation="fade-up" delay={100}>
                      <Card>
                        <CardHeader>
                          <CardTitle className="flex items-center gap-2">
                            <Settings className="w-5 h-5 text-blue-600" />
                            Pièces à Concevoir - SolidWorks 2025
                          </CardTitle>
                          <CardDescription>
                            Conception de 4 pièces mécaniques avec calculs de masse et assemblage final
                          </CardDescription>
                        </CardHeader>
                        <CardContent className="space-y-8">
                          
                          {/* Pièce 1 */}
                          <div className="border rounded-lg p-6 bg-gradient-to-r from-blue-50 to-cyan-50 dark:from-blue-950/20 dark:to-cyan-950/20">
                            <h3 className="text-xl font-semibold mb-4 flex items-center gap-2">
                              <span className="w-8 h-8 bg-blue-600 text-white rounded-full flex items-center justify-center text-sm font-bold">1</span>
                              Pièce 1 - Disque avec trous latéraux
                            </h3>
                            
                            <div className="grid md:grid-cols-2 gap-6 mb-4">
                              <div>
                                <Image 
                                  src="/2025-Team-IFRI-Docs/Documentation/semaine-1/mecanique/assets/imgs/picture_piece_1_trc.jpg" 
                                  alt="Pièce 1 - Vue technique" 
                                  width={300} 
                                  height={200} 
                                  className="rounded-lg border"
                                  unoptimized
                                />
                              </div>
                              <div>
                                <Image 
                                  src="/2025-Team-IFRI-Docs/Documentation/semaine-1/mecanique/assets/imgs/picture_piece_1_full.jpg" 
                                  alt="Pièce 1 - Vue complète" 
                                  width={300} 
                                  height={200} 
                                  className="rounded-lg border"
                                  unoptimized
                                />
                              </div>
                            </div>
                            
                            <div className="grid md:grid-cols-2 gap-6">
                              <div>
                                <h4 className="font-semibold mb-2">Caractéristiques</h4>
                                <ul className="text-sm space-y-1 text-muted-foreground">
                                  <li>• Matériau: Acier AISI 1020</li>
                                  <li>• Densité: 0,0079 g/mm³</li>
                                  <li>• Diamètre extérieur: 135.6mm</li>
                                  <li>• Trou central: Ø100mm</li>
                                  <li>• Masse calculée: <strong>2811.20 g</strong></li>
                                </ul>
                              </div>
                              <div>
                                <h4 className="font-semibold mb-2">Fichiers</h4>
                                <div className="space-y-2">
                                  <a href="/Documentation/semaine-1/mecanique/assets/pieces/piece_1/" 
                                     className="inline-flex items-center gap-2 text-sm text-blue-600 hover:text-blue-800">
                                    <Download className="w-4 h-4" />
                                    Télécharger Pièce 1 (.SLDPRT)
                                  </a>
                                </div>
                              </div>
                            </div>
                            
                            <div className="mt-4 p-4 bg-blue-50 dark:bg-blue-950/20 rounded-lg">
                              <h4 className="font-semibold mb-2 text-blue-800 dark:text-blue-200">Étapes de réalisation</h4>
                              <ol className="text-sm space-y-1 text-muted-foreground list-decimal list-inside">
                                <li>Création d'un nouveau projet</li>
                                <li>Réalisation de la partie circulaire : cercle Ø135.6 mm, deux cercles latéraux R40 mm espacés de 150 mm (75 mm de chaque côté de l'axe), reliés par arcs tangents</li>
                                <li>Extrusion du profil : profondeur 20 mm</li>
                                <li>Création du trou central : cercle Ø100 mm, coupe sur 20 mm</li>
                                <li>Création de la gorge annulaire : cercle Ø135.6 / Ø100 mm, extrusion 10 mm</li>
                                <li>Perçage des petits trous latéraux : Ø50 mm, traversants</li>
                                <li>Configurer matière : acier AISI 1020</li>
                                <li>Calcul de masse : 2811.1991 g</li>
                              </ol>
                            </div>
                          </div>

                          {/* Pièce 2 */}
                          <div className="border rounded-lg p-6 bg-gradient-to-r from-green-50 to-emerald-50 dark:from-green-950/20 dark:to-emerald-950/20">
                            <h3 className="text-xl font-semibold mb-4 flex items-center gap-2">
                              <span className="w-8 h-8 bg-green-600 text-white rounded-full flex items-center justify-center text-sm font-bold">2</span>
                              Pièce 2 - Cylindre par révolution
                            </h3>
                            
                            <div className="grid md:grid-cols-2 gap-6 mb-4">
                              <div>
                                <Image 
                                  src="/2025-Team-IFRI-Docs/Documentation/semaine-1/mecanique/assets/imgs/picture_piece_2_trc.jpg" 
                                  alt="Pièce 2 - Vue technique" 
                                  width={300} 
                                  height={200} 
                                  className="rounded-lg border"
                                  unoptimized
                                />
                              </div>
                              <div>
                                <Image 
                                  src="/2025-Team-IFRI-Docs/Documentation/semaine-1/mecanique/assets/imgs/picture_piece_2_full.jpg" 
                                  alt="Pièce 2 - Vue complète" 
                                  width={300} 
                                  height={200} 
                                  className="rounded-lg border"
                                  unoptimized
                                />
                              </div>
                            </div>
                            
                            <div className="grid md:grid-cols-2 gap-6">
                              <div>
                                <h4 className="font-semibold mb-2">Caractéristiques</h4>
                                <ul className="text-sm space-y-1 text-muted-foreground">
                                  <li>• Matériau: Aluminium Alliage 1060</li>
                                  <li>• Densité: 0.0027 g/mm³</li>
                                  <li>• Diamètres: Ø64, Ø55, Ø43, Ø38, Ø34, Ø20</li>
                                  <li>• Fonction: Révolution de base</li>
                                  <li>• Masse calculée: <strong>290.80 g</strong></li>
                                </ul>
                              </div>
                              <div>
                                <h4 className="font-semibold mb-2">Fichiers</h4>
                                <div className="space-y-2">
                                  <a href="/Documentation/semaine-1/mecanique/assets/pieces/piece_2/" 
                                     className="inline-flex items-center gap-2 text-sm text-green-600 hover:text-green-800">
                                    <Download className="w-4 h-4" />
                                    Télécharger Pièce 2 (.SLDPRT)
                                  </a>
                                </div>
                              </div>
                            </div>
                            
                            <div className="mt-4 p-4 bg-green-50 dark:bg-green-950/20 rounded-lg">
                              <h4 className="font-semibold mb-2 text-green-800 dark:text-green-200">Étapes de réalisation</h4>
                              <ol className="text-sm space-y-1 text-muted-foreground list-decimal list-inside">
                                <li>Création d'un nouveau projet</li>
                                <li>Esquisse du profil principal : plan de face, profil symétrique, respect des diamètres Ø64, Ø55, Ø43, Ø38, Ø34, Ø20 et longueurs correspondantes</li>
                                <li>Fonction Révolution : commande "Révolution de base" autour de l'axe vertical</li>
                                <li>Ajout des congés latéraux : rayon 64 mm</li>
                                <li>Création des perçages : coupe extrudée Ø20 et Ø34 sur les deux extrémités circulaires</li>
                                <li>Configurer matière : Aluminium Alliage 1060</li>
                                <li>Calcul de la masse : 290.80 g</li>
                              </ol>
                            </div>
                          </div>

                          {/* Pièce 3 */}
                          <div className="border rounded-lg p-6 bg-gradient-to-r from-orange-50 to-red-50 dark:from-orange-950/20 dark:to-red-950/20">
                            <h3 className="text-xl font-semibold mb-4 flex items-center gap-2">
                              <span className="w-8 h-8 bg-orange-600 text-white rounded-full flex items-center justify-center text-sm font-bold">3</span>
                              Pièce 3 - Forme trapézoïdale
                            </h3>
                            
                            <div className="grid md:grid-cols-2 gap-6 mb-4">
                              <div>
                                <Image 
                                  src="/2025-Team-IFRI-Docs/Documentation/semaine-1/mecanique/assets/imgs/picture_piece_3_trc.jpg" 
                                  alt="Pièce 3 - Vue technique" 
                                  width={300} 
                                  height={200} 
                                  className="rounded-lg border"
                                  unoptimized
                                />
                              </div>
                              <div>
                                <Image 
                                  src="/2025-Team-IFRI-Docs/Documentation/semaine-1/mecanique/assets/imgs/picture_piece_3_full.jpg" 
                                  alt="Pièce 3 - Vue complète" 
                                  width={300} 
                                  height={200} 
                                  className="rounded-lg border"
                                  unoptimized
                                />
                              </div>
                            </div>
                            
                            <div className="grid md:grid-cols-2 gap-6">
                              <div>
                                <h4 className="font-semibold mb-2">Caractéristiques</h4>
                                <ul className="text-sm space-y-1 text-muted-foreground">
                                  <li>• Matériau: Acier AISI 1020</li>
                                  <li>• Densité: 0,0079 g/mm³</li>
                                  <li>• Base: 150mm, Hauteur: 70mm</li>
                                  <li>• Évidements: 100×60×20mm</li>
                                  <li>• Masse calculée: <strong>1633.25 g</strong></li>
                                </ul>
                              </div>
                              <div>
                                <h4 className="font-semibold mb-2">Fichiers</h4>
                                <div className="space-y-2">
                                  <a href="/Documentation/semaine-1/mecanique/assets/pieces/piece_3/" 
                                     className="inline-flex items-center gap-2 text-sm text-orange-600 hover:text-orange-800">
                                    <Download className="w-4 h-4" />
                                    Télécharger Pièce 3 (.SLDPRT)
                                  </a>
                                </div>
                              </div>
                            </div>
                            
                            <div className="mt-4 p-4 bg-orange-50 dark:bg-orange-950/20 rounded-lg">
                              <h4 className="font-semibold mb-2 text-orange-800 dark:text-orange-200">Étapes de réalisation</h4>
                              <ol className="text-sm space-y-1 text-muted-foreground list-decimal list-inside">
                                <li>Création d'un nouveau projet</li>
                                <li>Esquisse du profil de base : trapèze fermé, base 150 mm, hauteur 70 mm, épaisseur 10 mm</li>
                                <li>Fonction Extrusion : profondeur 100 mm</li>
                                <li>Création des évidements : vides rectangulaires 100×60×20 mm à l'avant et à l'arrière</li>
                                <li>Ajout des blocs latéraux : 20×60×70 mm sur chaque extrémité</li>
                                <li>Configurer matière : acier AISI 1020</li>
                                <li>Calcul de la masse : 1633.25 g</li>
                              </ol>
                            </div>
                          </div>

                          {/* Pièce 4 */}
                          <div className="border rounded-lg p-6 bg-gradient-to-r from-purple-50 to-pink-50 dark:from-purple-950/20 dark:to-pink-950/20">
                            <h3 className="text-xl font-semibold mb-4 flex items-center gap-2">
                              <span className="w-8 h-8 bg-purple-600 text-white rounded-full flex items-center justify-center text-sm font-bold">4</span>
                              Pièce 4 - Géométrie complexe
                            </h3>
                            
                            <div className="grid md:grid-cols-2 gap-6 mb-4">
                              <div>
                                <Image 
                                  src="/2025-Team-IFRI-Docs/Documentation/semaine-1/mecanique/assets/imgs/picture_piece_4_trc.jpg" 
                                  alt="Pièce 4 - Vue technique" 
                                  width={300} 
                                  height={200} 
                                  className="rounded-lg border"
                                  unoptimized
                                />
                              </div>
                              <div>
                                <Image 
                                  src="/2025-Team-IFRI-Docs/Documentation/semaine-1/mecanique/assets/imgs/picture_piece_4_full.jpg" 
                                  alt="Pièce 4 - Vue complète" 
                                  width={300} 
                                  height={200} 
                                  className="rounded-lg border"
                                  unoptimized
                                />
                              </div>
                            </div>
                            
                            <div className="grid md:grid-cols-2 gap-6">
                              <div>
                                <h4 className="font-semibold mb-2">Caractéristiques</h4>
                                <ul className="text-sm space-y-1 text-muted-foreground">
                                  <li>• Matériau: Aluminium Alliage 1060</li>
                                  <li>• Densité: 0.0027 g/mm³</li>
                                  <li>• Dimensions: 100×50mm</li>
                                  <li>• Perçage: Ø30mm, Évidement: Ø24mm</li>
                                  <li>• Masse calculée: <strong>297.29 g</strong></li>
                                </ul>
                              </div>
                              <div>
                                <h4 className="font-semibold mb-2">Fichiers</h4>
                                <div className="space-y-2">
                                  <a href="/Documentation/semaine-1/mecanique/assets/pieces/piece_4/" 
                                     className="inline-flex items-center gap-2 text-sm text-purple-600 hover:text-purple-800">
                                    <Download className="w-4 h-4" />
                                    Télécharger Pièce 4 (.SLDPRT)
                                  </a>
                                </div>
                              </div>
                            </div>
                            
                            <div className="mt-4 p-4 bg-purple-50 dark:bg-purple-950/20 rounded-lg">
                              <h4 className="font-semibold mb-2 text-purple-800 dark:text-purple-200">Étapes de réalisation</h4>
                              <ol className="text-sm space-y-1 text-muted-foreground list-decimal list-inside">
                                <li>Création d'un nouveau projet</li>
                                <li>Esquisse de base : dimensions 100×50 mm</li>
                                <li>Extrusion principale : hauteur 25 mm</li>
                                <li>Perçage cylindrique : Ø30 mm traversant, Ø24 mm en surface</li>
                                <li>Géométrie inclinée : angle 45°, congés R2</li>
                                <li>Section transversale complexe : hauteurs 10-15-25 mm, raccords R2, découpes et évidements</li>
                                <li>Finitions : congés, chanfreins, vérification</li>
                                <li>Configurer matière : Aluminium Alliage 1060</li>
                                <li>Calcul de la masse : 297.29 g</li>
                              </ol>
                            </div>
                          </div>

                        </CardContent>
                      </Card>
                    </AnimatedSection>

                    {/* Assemblage */}
                    <AnimatedSection animation="fade-up" delay={200}>
                      <Card>
                        <CardHeader>
                          <CardTitle className="flex items-center gap-2">
                            <Layers className="w-5 h-5 text-indigo-600" />
                            Assemblage Final - Pince Mécanique
                          </CardTitle>
                          <CardDescription>
                            Assemblage des pièces pour former une pince mécanique fonctionnelle
                          </CardDescription>
                        </CardHeader>
                        <CardContent>
                          <div className="grid md:grid-cols-2 gap-6 mb-6">
                            <div>
                              <Image 
                                src="/2025-Team-IFRI-Docs/Documentation/semaine-1/mecanique/assets/imgs/Piece_assemblage_trc.jpg" 
                                alt="Assemblage - Vue détaillée" 
                                width={400} 
                                height={300} 
                                className="rounded-lg border w-full"
                                unoptimized
                              />
                            </div>
                            <div>
                              <Image 
                                src="/2025-Team-IFRI-Docs/Documentation/semaine-1/mecanique/assets/imgs/Piece_assemblage.jpg" 
                                alt="Assemblage - Vue d'ensemble" 
                                width={400} 
                                height={300} 
                                className="rounded-lg border w-full"
                                unoptimized
                              />
                            </div>
                          </div>
                          
                          <div className="grid md:grid-cols-2 gap-6">
                            <div>
                              <h4 className="font-semibold mb-3">Coordonnées du centre de masse</h4>
                              <div className="space-y-3">
                                <div className="bg-muted/50 p-3 rounded">
                                  <h5 className="font-medium text-sm">Position minimale</h5>
                                  <p className="text-sm text-muted-foreground">X: -29.15mm, Y: 0.16mm, Z: 19.86mm</p>
                                </div>
                                <div className="bg-muted/50 p-3 rounded">
                                  <h5 className="font-medium text-sm">Position maximale</h5>
                                  <p className="text-sm text-muted-foreground">X: -25.78mm, Y: 0.06mm, Z: 19.86mm</p>
                                </div>
                              </div>
                            </div>
                            <div>
                              <h4 className="font-semibold mb-3">Fichiers d'assemblage</h4>
                              <div className="space-y-2">
                                <a href="/Documentation/semaine-1/mecanique/assets/zips/" 
                                   className="inline-flex items-center gap-2 text-sm text-indigo-600 hover:text-indigo-800">
                                  <Download className="w-4 h-4" />
                                  Télécharger Assemblage (.SLDASM)
                                </a>
                              </div>
                            </div>
                          </div>
                        </CardContent>
                      </Card>
                    </AnimatedSection>

                    {/* Vidéo */}
                    <AnimatedSection animation="fade-up" delay={300}>
                      <Card>
                        <CardHeader>
                          <CardTitle className="flex items-center gap-2">
                            <Play className="w-5 h-5 text-red-600" />
                            Démonstration Vidéo
                          </CardTitle>
                          <CardDescription>
                            Pince mécanique en mouvement
                          </CardDescription>
                        </CardHeader>
                        <CardContent>
                          <div className="aspect-video rounded-lg overflow-hidden border">
                            <iframe 
                              src="https://player.vimeo.com/video/1092855533?h=480fde870c" 
                              width="100%" 
                              height="100%" 
                              frameBorder="0" 
                              allow="autoplay; fullscreen; picture-in-picture" 
                              allowFullScreen
                              className="w-full h-full"
                            />
                          </div>
                        </CardContent>
                      </Card>
                    </AnimatedSection>
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