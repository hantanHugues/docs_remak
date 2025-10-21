"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Code, BookOpen, GitBranch, PlayCircle, FolderTree, BarChart2, Zap, Settings, ShieldAlert, FileText
} from "lucide-react";
import Link from "next/link";

export default function PathfindingAStarDocPage() {
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
                    <span>Recherche de chemin A*</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <GitBranch className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Recherche de chemin A* pour robot sur grille</h1>
                      <p className="text-muted-foreground">Solution de pathfinding pour un robot évoluant sur une grille à obstacles.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Python</Badge>
                    <Badge variant="outline">ROS 2</Badge>
                    <Badge variant="outline">Gazebo</Badge>
                    <Badge variant="outline">Algorithme A*</Badge>
                   </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
            <div className="container mx-auto px-4">
              <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
                <Link href="/docs/semaine-3/it">
                  <Button variant="ghost" size="sm">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    IT Semaine 3
                  </Button>
                </Link>
                <Link href="/docs/semaine-3/it/algorithme-cartographie-nav2">
                  <Button variant="ghost" size="sm">
                    Cartographie Nav2
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
                      Présentation
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        Ce projet propose une solution efficace de pathfinding pour un robot évoluant sur une grille à obstacles, en utilisant l'algorithme A* (A-star). Il est conçu pour être intégré dans un environnement de simulation ROS2 et Gazebo, avec visualisation possible dans RViz 2. L'objectif est de permettre au robot de rejoindre une cible tout en évitant les obstacles, de manière optimale et reproductible.
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <Separator className="border-gray-200 dark:border-gray-700" />
              
              <div className="space-y-12">
              
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <Settings className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        1. Configuration ROS2 et Gazebo
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6">
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm mb-4">
                        <p className="text-gray-600 dark:text-gray-400">
                          <strong>Note :</strong> Ce projet fournit la logique de pathfinding. L'intégration complète avec ROS2 et Gazebo nécessite d'adapter les scripts pour publier/écouter sur les bons topics ROS2 et interfacer avec les plugins de simulation. Voici les grandes lignes pour une intégration typique :
                        </p>
                      </div>
                      <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                        <li><strong>ROS2</strong> :
                          <ul className="list-['-_'] pl-5 mt-1 text-sm text-gray-600 dark:text-gray-400">
                            <li>Créez un package ROS2 (ex : <code>robot_pathfinding</code>).</li>
                            <li>Adaptez <code>main_astar.py</code> pour recevoir la grille, la position initiale et la cible via des topics ou services ROS2 (ex : <code>/map</code>, <code>/initial_pose</code>, <code>/goal_pose</code>).</li>
                            <li>Publiez le chemin trouvé sur un topic (ex : <code>/planned_path</code>).</li>
                          </ul>
                        </li>
                        <li><strong>Gazebo</strong> :
                          <ul className="list-['-_'] pl-5 mt-1 text-sm text-gray-600 dark:text-gray-400">
                            <li>Utilisez un monde Gazebo avec une carte d'occupation (OccupancyGrid) correspondant à la matrice utilisée.</li>
                            <li>Le robot doit être contrôlé pour suivre le chemin calculé (via un contrôleur ROS2).</li>
                          </ul>
                        </li>
                        <li><strong>RViz 2</strong> :
                          <ul className="list-['-_'] pl-5 mt-1 text-sm text-gray-600 dark:text-gray-400">
                            <li>Visualisez la carte, la position du robot, la cible et le chemin planifié en important les topics correspondants.</li>
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
                        <Zap className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        2. Algorithme de pathfinding : A*
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 space-y-4">
                      <p className="text-gray-700 dark:text-gray-300">L'algorithme A* est un algorithme de recherche informée qui combine le coût réel du chemin parcouru et une estimation heuristique du coût restant pour atteindre la cible. Il garantit de trouver le chemin le plus court si l'heuristique est admissible.</p>
                      <h3 className="font-semibold text-gray-800 dark:text-gray-200">Fonctionnement détaillé :</h3>
                      <ul className="list-disc pl-5 space-y-1 text-sm text-gray-600 dark:text-gray-400">
                        <li><strong>État</strong> : chaque case de la grille est un état (x, y).</li>
                        <li><strong>Actions</strong> : le robot peut se déplacer dans 8 directions (haut, bas, gauche, droite, et diagonales).</li>
                        <li><strong>Coût</strong> : 1 pour un déplacement orthogonal, √2 pour un déplacement diagonal.</li>
                        <li><strong>Heuristique</strong> : distance octile (adaptée à la 8-connexité).</li>
                        <li><strong>Gestion des obstacles</strong> : Les cases non nulles sont des obstacles infranchissables.</li>
                      </ul>
                      <h3 className="font-semibold text-gray-800 dark:text-gray-200">Justification du choix :</h3>
                      <ul className="list-disc pl-5 space-y-1 text-sm text-gray-600 dark:text-gray-400">
                        <li><strong>A*</strong> est reconnu pour son efficacité et son optimalité.</li>
                        <li>L'heuristique (distance octile) est parfaitement adaptée et accélère la recherche.</li>
                        <li>La gestion stricte des obstacles garantit la sécurité.</li>
                      </ul>
                    </CardContent>
                  </Card>
                </AnimatedSection>
                
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <ShieldAlert className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        3. Gestion des obstacles
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6">
                      <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                        <li>Les obstacles sont définis dans la matrice d'occupation (<code>1</code> = obstacle, <code>0</code> = libre).</li>
                        <li>Le robot vérifie à chaque étape que la case cible est libre.</li>
                        <li>Pour les déplacements en diagonale, le robot vérifie que le passage n'est pas bloqué par deux obstacles adjacents (pas de « corner cutting »).</li>
                        <li>Cette gestion assure que le robot ne tente jamais de traverser un obstacle, même partiellement.</li>
                      </ul>
                    </CardContent>
                  </Card>
                </AnimatedSection>

                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <BarChart2 className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        4. Simulation et visualisation (Gazebo & RViz 2)
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 space-y-4">
                      <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                        <li><strong>Simulation dans Gazebo</strong> : Importez la carte, placez le robot, et utilisez le chemin calculé pour générer des commandes.</li>
                        <li><strong>Visualisation dans RViz 2</strong> : Affichez la carte (<code>/map</code>), la position (<code>/odom</code>), la cible, et le chemin (<code>/planned_path</code>).</li>
                        <li><strong>Conseils</strong> : Vérifiez la correspondance entre la matrice et la carte. Utilisez des couleurs distinctes pour la lisibilité.</li>
                      </ul>
                    </CardContent>
                  </Card>
                </AnimatedSection>

                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <FileText className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        5. Documentation de l'algorithme, des choix techniques et des résultats
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 space-y-4">
                      <h3 className="font-semibold text-gray-800 dark:text-gray-200">Choix techniques</h3>
                      <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400">
                        <li><strong>Python</strong> pour la rapidité de prototypage.</li>
                        <li><strong>A*</strong> pour l'optimalité sur grille.</li>
                        <li><strong>8-connexité</strong> pour des déplacements naturels.</li>
                        <li><strong>Gestion stricte des obstacles</strong> pour la sécurité.</li>
                      </ul>
                      <h3 className="font-semibold text-gray-800 dark:text-gray-200">Résultats obtenus</h3>
                      <p className="text-gray-700 dark:text-gray-300">Le programme affiche le chemin, le nombre de nœuds explorés et le temps de calcul. Le robot évite systématiquement tous les obstacles.</p>
                      <h4 className="font-medium text-gray-900 dark:text-gray-100">Exemple de sortie :</h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`Chemin trouvé (liste d'états (x,y)) :
  Étape 0: (0, 0)
  Étape 1: (1, 1)
  ...
Nombre de nœuds explorés par A*: 23
Temps de calcul A*: 0.0021 s`}</code></pre></div>
                    </CardContent>
                  </Card>
                </AnimatedSection>
                
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        6. Utilisation rapide
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6">
                      <ol className="list-decimal pl-5 space-y-4 text-gray-700 dark:text-gray-300">
                        <li>Modifiez la grille, la position initiale et la cible dans <code>main_astar.py</code>.</li>
                        <li>Lancez le programme :<div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 my-2 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`python main_astar.py`}</code></pre></div></li>
                        <li>Intégrez le code dans votre package ROS2 pour une utilisation en simulation.</li>
                      </ol>
                    </CardContent>
                  </Card>
                </AnimatedSection>
                
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <FolderTree className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        7. Structure des fichiers
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6">
                      <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                        <li><strong>main_astar.py</strong> : point d'entrée, configuration et lancement de la recherche.</li>
                        <li><strong>robot_search.py</strong> : définition du problème de déplacement sur grille.</li>
                        <li><strong>search.py</strong> : algorithmes de recherche (A*, etc.).</li>
                        <li><strong>utils.py</strong> : fonctions utilitaires.</li>
                      </ul>
                    </CardContent>
                  </Card>
                </AnimatedSection>
                
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <FileText className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        8. Documentation détaillée des fichiers du projet
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 space-y-4">
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100">1. <code>main_astar.py</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mt-2"><strong>Rôle :</strong> Point d'entrée, configuration de la grille, lancement de A*, affichage des résultats.</p>
                      </div>
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100">2. <code>robot_search.py</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mt-2"><strong>Rôle :</strong> Définit le problème de déplacement, gère la 8-connexité, la vérification des obstacles (y compris "corner cutting"), et fournit l'heuristique (distance octile).</p>
                      </div>
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100">3. <code>search.py</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mt-2"><strong>Rôle :</strong> Contient les algorithmes de recherche génériques (A*, etc.) et les structures de base (<code>Problem</code>, <code>Node</code>).</p>
                      </div>
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100">4. <code>utils.py</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mt-2"><strong>Rôle :</strong> Fournit des fonctions utilitaires pour les mathématiques, la manipulation de listes, et les structures de données (files de priorité) utilisées par les algorithmes de recherche.</p>
                      </div>
                    </CardContent>
                  </Card>
                </AnimatedSection>
              </div>

              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-3/it">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    IT Semaine 3
                  </Button>
                </Link>
                <Link href="/docs/semaine-3/it/algorithme-cartographie-nav2">
                  <Button>
                    Cartographie Nav2
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