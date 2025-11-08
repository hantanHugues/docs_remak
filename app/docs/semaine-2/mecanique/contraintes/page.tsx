"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, HardHat, Cog, BookOpen, Download, PlayCircle, FolderArchive, Target, CheckCircle, BarChart2, AlertTriangle, ExternalLink, Settings, Layers, Calculator
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';

export default function MechanicsDocPage() {
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
                      Documentation
                    </Link>
                    <span>/</span>
                    <span>Mécanique</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <HardHat className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Documentation Mécanique</h1>
                      <p className="text-muted-foreground"><strong>Test 2 – Niveau Intermédiaire</strong></p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Mécanique</Badge>
                    <Badge variant="outline">SolidWorks</Badge>
                    <Badge variant="outline">CAO</Badge>
                    <Badge variant="outline">Intermédiaire</Badge>
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
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Introduction et Contexte
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300 mb-4">
                      Le pôle mécanique a pour mission de concevoir, modéliser et valider des composants assurant la solidité et la précision du robot. Cette documentation vise à :
                    </p>
                    <ul className="list-disc pl-5 space-y-2 text-gray-600 dark:text-gray-400">
                      <li>Décrire en détail la démarche de modélisation paramétrique sous <strong className="font-semibold text-primary">SolidWorks 2025</strong>.</li>
                      <li>Fournir une traçabilité complète des choix techniques, des calculs et des validations.</li>
                      <li>Servir de référence pour la maintenance et l’évolution future des pièces.</li>
                    </ul>
                    <p className="mt-4 bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                      <strong>Usage</strong> : Ce document est destiné aux membres de l’équipe, au jury de la compétition et aux futurs référents techniques.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={50}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Target className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Objectifs du Projet
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ol className="list-decimal pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li>Reproduire fidèlement une pièce 2D fournie selon un document de spécification technique.</li>
                      <li>Implémenter trois variables globales (A, B, C) pour automatiser les variations dimensionnelles.</li>
                      <li>Calculer la masse des pièces pour chaque jeu de paramètres.</li>
                      <li>Appliquer découpe, congés et assemblage conformément aux spécifications.</li>
                      <li>Répondre aux questions de position du centre de masse pour deux configurations angulaires.</li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <FolderArchive className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Portée et Livrables
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li><strong>Fichiers pièces</strong> : <code>.SLDPRT</code> pour chaque configuration.</li>
                      <li><strong>Fichiers assemblage</strong> : <code>.SLDASM</code> validés et annotés.</li>
                      <li><strong>Captures d’écran</strong> : Vues d’esquisse, isométriques, plan et propriétés de masse.</li>
                      <li><strong>Vidéos</strong> : Tutoriel de modélisation (2 minutes).</li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      Matériel, Outils et Environnement
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="overflow-x-auto">
                      <table className="w-full text-sm">
                        <thead>
                          <tr className="bg-gray-50 dark:bg-gray-800">
                            <th className="px-4 py-2 text-left font-medium text-gray-900 dark:text-gray-100 border border-gray-200 dark:border-gray-700">Élément</th>
                            <th className="px-4 py-2 text-left font-medium text-gray-900 dark:text-gray-100 border border-gray-200 dark:border-gray-700">Détail</th>
                          </tr>
                        </thead>
                        <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
                          <tr><td className="px-4 py-2 border border-gray-200 dark:border-gray-700">Logiciel</td><td className="px-4 py-2 border border-gray-200 dark:border-gray-700">SolidWorks 2025</td></tr>
                          <tr><td className="px-4 py-2 border border-gray-200 dark:border-gray-700">Matériau</td><td className="px-4 py-2 border border-gray-200 dark:border-gray-700">Acier AISI 1020 (densité 0.0079 g/mm³)</td></tr>
                          <tr><td className="px-4 py-2 border border-gray-200 dark:border-gray-700">Unités</td><td className="px-4 py-2 border border-gray-200 dark:border-gray-700">MMGS (mm, g, s), précision ±0.01 mm/g</td></tr>
                          <tr><td className="px-4 py-2 border border-gray-200 dark:border-gray-700">Plugins/Librairies</td><td className="px-4 py-2 border border-gray-200 dark:border-gray-700">Toolbox standard, équations globales</td></tr>
                        </tbody>
                      </table>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Cog className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Conception et Réalisation des Pièces
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <div>
                      <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">5.1. Concept et Cahier des Charges</h3>
                      <ul className="list-disc pl-5 mt-2 text-sm text-gray-600 dark:text-gray-400">
                        <li>Pièce de support modulable, assurant la rigidité sous charge statique.</li>
                        <li>Contraintes : trou Ø14 mm, congés internes/externes (R5, R29), angles critiques (45°, 10°).</li>
                        <li>Objectifs de flexibilité via variables : A (largeur), B (hauteur), C (épaisseur).</li>
                      </ul>
                    </div>
                    <div>
                      <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">5.2. Modélisation Paramétrique</h3>
                      <p className="text-gray-700 dark:text-gray-300 mt-2">Variables globales définies dans l’onglet « Équations ». Liaisons intelligentes : « Jusqu’à suivant » pour profondeur, « A » pour largeur de base, etc.</p>
                    </div>
                    <div>
                      <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">5.3. Processus de modélisation</h3>
                      <div className="space-y-4 mt-2">
                        <div>
                          <h4 className="font-medium text-blue-700 dark:text-blue-300">5.3.1. Préparation du Fichier</h4>
                          <ol className="list-decimal pl-5 mt-2 text-sm text-gray-600 dark:text-gray-400">
                            <li>Ouvrir un nouveau document pièce, format MMGS.</li>
                            <li>Définir tolérances par défaut dans Propriétés du document.</li>
                            <li>Enregistrer sous <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">modele_base.SLDPRT</code>.</li>
                          </ol>
                        </div>
                        
                        <div>
                          <h4 className="font-medium text-blue-700 dark:text-blue-300">5.3.2. Esquisse 2D Détaillée</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">Plan de face (Front Plane). Esquisse des profils :</p>
                          <ol className="list-decimal pl-5 text-sm text-gray-600 dark:text-gray-400">
                            <li>Ligne principale de base (longueur A).</li>
                            <li>Arcs et cercles (incl. trou Ø14 mm centré).</li>
                            <li>Positionnement des congés internes (R5) et externes (R29).</li>
                            <li>Cotes angulaires (45°, 10°) appliquées via contrainte de cote.</li>
                          </ol>
                        </div>
                        
                        <div>
                          <h4 className="font-medium text-blue-700 dark:text-blue-300">5.3.3. Définition des Variables Globales</h4>
                          <div className="bg-blue-50 dark:bg-blue-900/20 p-3 rounded mt-2">
                            <p className="text-sm"><strong>A</strong> = 81 / 84 mm<br/><strong>B</strong> = 57 / 59 mm<br/><strong>C</strong> = 43 / 45 mm</p>
                          </div>
                        </div>
                        
                        <div>
                          <h4 className="font-medium text-blue-700 dark:text-blue-300">5.3.4. Opérations de Volume (Extrusion, Congés)</h4>
                          <ol className="list-decimal pl-5 mt-2 text-sm text-gray-600 dark:text-gray-400">
                            <li><strong>Extrusion Bossage/Base</strong> sur épaisseur C.</li>
                            <li><strong>Découpe</strong> : extrémités profilées selon A, B.</li>
                            <li><strong>Congés</strong> :
                              <ul className="list-disc pl-5 mt-1">
                                <li>R5 sur arêtes internes avant enlèvement latéral.</li>
                                <li>R29 sur bords externes en finition.</li>
                              </ul>
                            </li>
                          </ol>
                        </div>
                        
                        <div>
                          <h4 className="font-medium text-blue-700 dark:text-blue-300">5.3.5. Contrôles et Validation Géométrique</h4>
                          <ul className="list-disc pl-5 mt-2 text-sm text-gray-600 dark:text-gray-400">
                            <li>Utiliser la fonction « Vérifier géométrie » pour détecter surfaces non-manifold.</li>
                            <li>Visualiser la masse via Propriétés → Propriétés de masse.</li>
                            <li>Comparer contre valeurs attendues.</li>
                          </ul>
                        </div>
                        
                        <div>
                          <h4 className="font-medium text-blue-700 dark:text-blue-300">5.3.6. Exportation des Pièces</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400 mt-2">Enregistrer chaque variante sous <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">partieX.SLDPRT</code>.</p>
                        </div>
                      </div>
                    </div>
                    <div>
                      <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">5.4. Illustrations et Captures d’Écran</h3>
                      <p className="text-gray-700 dark:text-gray-300 mt-2">Organisation des images dans un dossier <code>images/</code> avec légendes et repères.</p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Résultats et Réponses aux Questions du Test
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <div>
                      <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">7.1. Partie 1 – Variations de A, B, C</h3>
                      <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_piece_partie1.png" alt="Vue détaillée de la pièce - Partie 1" width={700} height={400} className="rounded-md border my-4" unoptimized/>
                      <div className="grid md:grid-cols-2 gap-4">
                        <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-4">
                          <h4 className="font-semibold mb-2">Question A</h4>
                          <ul className="text-sm space-y-1">
                            <li>Valeurs : <strong>A = 81 mm, B = 57 mm, C = 43 mm</strong></li>
                            <li>Masse : <strong>939.54 grammes</strong></li>
                            <li><a href="/Documentation/semaine-2/mecanique/assets/pieces/piece1/piece_partie1_a.SLDPRT" className="text-blue-600 hover:underline inline-flex items-center gap-1"><Download className="w-3 h-3"/>Télécharger la pièce</a></li>
                          </ul>
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_mass_properties_partie1_a.jpg" alt="Propriétés de masse - Partie1 A" width={200} height={150} className="rounded-md border mt-2" unoptimized/>
                        </div>
                        <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-4">
                          <h4 className="font-semibold mb-2">Question B</h4>
                          <ul className="text-sm space-y-1">
                            <li>Valeurs : <strong>A = 84 mm, B = 59 mm, C = 45 mm</strong></li>
                            <li>Masse : <strong>1032.32 grammes</strong></li>
                            <li><a href="/Documentation/semaine-2/mecanique/assets/pieces/piece1/piece_partie1_b.SLDPRT" className="text-blue-600 hover:underline inline-flex items-center gap-1"><Download className="w-3 h-3"/>Télécharger la pièce</a></li>
                          </ul>
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_mass_properties_partie1_b.png" alt="Propriétés de masse - Partie1 B" width={200} height={150} className="rounded-md border mt-2" unoptimized/>
                        </div>
                      </div>
                    </div>
                    <div>
                      <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">7.2. Partie 2 – Cas supplémentaire</h3>
                      <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_piece_partie2.png" alt="Vue détaillée de la pièce - Partie 2" width={700} height={400} className="rounded-md border my-4" unoptimized/>
                      <div className="flex items-center gap-4 mb-4">
                        <p><strong>Masse : 628.18 grammes</strong></p>
                        <a href="/Documentation/semaine-2/mecanique/assets/pieces/piece2/piece_partie2.SLDPRT" className="text-blue-600 hover:underline inline-flex items-center gap-1">
                          <Download className="w-4 h-4"/>Télécharger la pièce
                        </a>
                      </div>
                      <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_mass_properties_partie2.png" alt="Propriétés de masse - Partie2" width={300} height={200} className="rounded-md border" unoptimized/>
                    </div>
                    <div>
                      <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">7.3. Partie 3 – Exemple de réduction</h3>
                      <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_piece_partie3.png" alt="Vue détaillée de la pièce - Partie 3" width={700} height={400} className="rounded-md border my-4" unoptimized/>
                      <div className="flex items-center gap-4 mb-4">
                        <p><strong>Masse : 432.58 grammes</strong></p>
                        <a href="/Documentation/semaine-2/mecanique/assets/pieces/piece3/picture_piece_partie3.SLDPRT" className="text-blue-600 hover:underline inline-flex items-center gap-1">
                          <Download className="w-4 h-4"/>Télécharger la pièce
                        </a>
                      </div>
                      <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_mass_properties_partie3.png" alt="Propriétés de masse - Partie3" width={300} height={200} className="rounded-md border" unoptimized/>
                    </div>
                    <div>
                      <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200">7.4. Assemblage et Centre de Masse</h3>
                      <p><strong>Centre de masse</strong> :</p>
                      <ul className="text-sm">
                        <li>Config. A (A=25°, B=125°, C=130°): <strong>(X=327.67, Y=-98.39, Z=-102.91)</strong></li>
                        <li>Config. B (A=30°, B=115°, C=135°): <strong>(X=348.66, Y=-88.48, Z=-91.40)</strong></li>
                      </ul>
                      <div className="grid md:grid-cols-2 gap-4 my-4">
                        <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_assemblage_a.png" alt="Assemblage - Question A" width={300} height={200} className="rounded-md border" unoptimized/>
                        <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_assemblage_b.png" alt="Assemblage - Question B" width={300} height={200} className="rounded-md border" unoptimized/>
                      </div>
                      <div className="grid md:grid-cols-2 gap-4">
                        <div>
                          <p className="mb-2">a) Coordonnées pour A=25°; B=125°; C=130° : (X=<strong>327.67</strong>, Y=<strong>-98.39</strong>, Z=<strong>-102.91</strong>)</p>
                          <a href="/Documentation/semaine-2/mecanique/assets/pieces/assemblage/Question_A.zip" className="text-blue-600 hover:underline inline-flex items-center gap-1 mb-2">
                            <Download className="w-4 h-4"/>Télécharger assemblage - Question A
                          </a>
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_mass_properties_a.png" alt="Propriétés masse - Question A" width={300} height={200} className="rounded-md border mt-2" unoptimized/>
                        </div>
                        <div>
                          <p className="mb-2">b) Coordonnées pour A=30°; B=115°; C=135° : (X=<strong>348.66</strong>, Y=<strong>-88.48</strong>, Z=<strong>-91.40</strong>)</p>
                          <a href="/Documentation/semaine-2/mecanique/assets/pieces/assemblage/Question_B.zip" className="text-blue-600 hover:underline inline-flex items-center gap-1 mb-2">
                            <Download className="w-4 h-4"/>Télécharger assemblage - Question B
                          </a>
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_mass_properties_b.png" alt="Propriétés masse - Question B" width={300} height={200} className="rounded-md border mt-2" unoptimized/>
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
                      <AlertTriangle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Difficultés Rencontrées et Solutions
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="overflow-x-auto">
                      <table className="w-full text-sm border-collapse">
                        <thead>
                          <tr className="bg-orange-100 dark:bg-orange-900/30">
                            <th className="border p-3 text-left font-medium">Problème</th>
                            <th className="border p-3 text-left font-medium">Analyse détaillée</th>
                            <th className="border p-3 text-left font-medium">Solution mise en place</th>
                          </tr>
                        </thead>
                        <tbody>
                          <tr>
                            <td className="border p-3">Application des congés dans espaces étroits</td>
                            <td className="border p-3">Impossibilité de sélectionner toutes les arêtes sans erreurs de géométrie</td>
                            <td className="border p-3">Séparation en deux opérations : congés internes d'abord, puis externes</td>
                          </tr>
                          <tr>
                            <td className="border p-3">Validation du centre de masse</td>
                            <td className="border p-3">Position non-conforme aux spécifications angulaires</td>
                            <td className="border p-3">Calibration via mesures sur assemblage simulé, ajustement des axes de référence</td>
                          </tr>
                        </tbody>
                      </table>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Section Vidéo et Médias
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li><strong>Vidéo tutoriel</strong> : démonstration des étapes clés (2 min).</li>
                      <li><strong>Liens</strong> :
                        <ul className="list-['-_'] pl-5 mt-1 text-blue-600 dark:text-blue-400">
                          <li><Link href="https://youtu.be/" className="hover:underline inline-flex items-center gap-1">YouTube <ExternalLink className="w-3 h-3"/></Link></li>
                          <li><Link href="https://github.com/TekBot-Robotics-Challenge/2025-Team-IFRI-Docs" className="hover:underline inline-flex items-center gap-1">Repos GitHub <ExternalLink className="w-3 h-3"/></Link></li>
                        </ul>
                      </li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      Glossaire
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li><strong>Bossage/Base</strong> : extrusion élémentaire créant un volume.</li>
                      <li><strong>Congé</strong> : arrondi appliqué sur une arête.</li>
                      <li><strong>Équations globales</strong> : variables paramétrant le modèle.</li>
                      <li><strong>Manifold</strong> : surface continue sans discontinuité.</li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      Références et Liens Utiles
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li>Manuel SolidWorks 2025 – Chap. Équations et Variables</li>
                      <li>Tutoriels suivis :
                        <ul className="list-['-_'] pl-5 mt-1 text-blue-600 dark:text-blue-400">
                          <li><Link href="https://youtu.be/PQHjY9_b94w?si=Ah0PxEUdaubB3jMf" className="hover:underline inline-flex items-center gap-1">https://youtu.be/PQHjY9_b94w <ExternalLink className="w-3 h-3"/></Link></li>
                          <li><Link href="https://youtu.be/ESkXkDUmsNc?si=8zdjJDQK-7fBHAwG" className="hover:underline inline-flex items-center gap-1">https://youtu.be/ESkXkDUmsNc <ExternalLink className="w-3 h-3"/></Link></li>
                        </ul>
                      </li>
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