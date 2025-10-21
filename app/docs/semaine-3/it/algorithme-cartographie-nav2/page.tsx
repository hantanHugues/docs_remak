"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Code, BookOpen, GitBranch, PlayCircle, Bot, Zap, Settings, AlertTriangle, Download, Compass, Search, Target
} from "lucide-react";
import Link from "next/link";

export default function Nav2AutoExplorerDocPage() {
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
                    <span>Exploration Autonome Nav2</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Compass className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Documentation du script d'exploration autonome pour Nav2</h1>
                      <p className="text-muted-foreground">Stratégie d'exploration par frontières pour cartographier un environnement inconnu.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">ROS 2</Badge>
                    <Badge variant="outline">Nav2</Badge>
                    <Badge variant="outline">Python</Badge>
                    <Badge variant="outline">SLAM</Badge>
                   </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
            <div className="container mx-auto px-4">
              <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
                <Link href="/docs/semaine-3/it/algorithme-pathfinding">
                  <Button variant="ghost" size="sm">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Pathfinding
                  </Button>
                </Link>
                <Link href="/docs/semaine-3/it">
                  <Button variant="ghost" size="sm">
                    IT Semaine 3
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
                      <Zap className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      But du script
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        Ce script implémente une stratégie <strong>d'exploration autonome par frontières</strong> pour un robot utilisant Nav2. Son objectif est de cartographier un environnement inconnu (comme un labyrinthe) de manière entièrement automatique.
                      </p>
                      <p className="mt-4">Le principe est le suivant :</p>
                      <ol className="list-decimal pl-5 mt-2 space-y-2">
                        <li>Le robot analyse en permanence la carte construite par un algorithme de SLAM.</li>
                        <li>Il identifie les <strong>"frontières"</strong> : des zones connues et accessibles situées juste à côté de zones encore inconnues.</li>
                        <li>Il choisit la frontière la plus prometteuse (ici, la plus proche) comme nouvel objectif.</li>
                        <li>Il utilise Nav2 pour naviguer jusqu'à cet objectif, révélant ainsi une nouvelle partie de la carte.</li>
                        <li>Le processus se répète jusqu'à ce qu'il n'y ait plus de frontières à explorer, signifiant que la carte est complète.</li>
                      </ol>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <Separator className="border-gray-200 dark:border-gray-700" />

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Comment utiliser ce script ?
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ol className="list-decimal pl-5 space-y-4 text-gray-700 dark:text-gray-300">
                      <li>
                        <strong>Lancement de la simulation complète</strong> : Démarrez votre environnement de simulation (Gazebo), le robot, le nœud SLAM (<code>slam_toolbox</code>) et la pile de navigation Nav2.
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 my-2 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                          <pre className="text-sm text-gray-100"><code>{`# Exemple de commande de lancement
ros2 launch mon_robot_config simulation_avec_nav2.launch.py`}</code></pre>
                        </div>
                      </li>
                      <li>
                        <strong>Lancement du script d'exploration</strong> : Dans un nouveau terminal (après avoir sourcé votre workspace), lancez ce nœud.
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 my-2 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                          <pre className="text-sm text-gray-100"><code>{`ros2 run mon_paquet_exploration nom_de_l_executable`}</code></pre>
                        </div>
                      </li>
                      <li>
                        <strong>Observation</strong> : Dans RViz, vous devriez voir le robot recevoir des objectifs de navigation (flèches vertes) et se déplacer de manière autonome pour explorer la carte. La carte se complétera progressivement.
                      </li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Code className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Explication détaillée du code
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-3">1. <strong>Initialisation (`__init__`)</strong></h3>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`class Nav2AutoExplorerImproved(Node):
    def __init__(self):
        super().__init__('nav2_auto_explorer_improved')
        # ... Définition des publishers, subscribers, et du TF listener ...
        self.timer = self.create_timer(self.exploration_timer_period, self.explore)`}</code></pre></div>
                      <p className="text-sm font-semibold text-gray-700 dark:text-gray-300 mb-2">Rôles des composants :</p>
                      <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400 space-y-1">
                          <li><code>goal_pub</code> : Pour envoyer des objectifs de navigation (`PoseStamped`) à Nav2.</li>
                          <li><code>map_sub</code> : Pour recevoir la carte (`OccupancyGrid`) en temps réel depuis SLAM.</li>
                          <li><code>tf_buffer</code> et <code>tf_listener</code> : Outil essentiel pour obtenir la position exacte du robot.</li>
                      </ul>
                      <p className="text-sm font-semibold text-gray-700 dark:text-gray-300 mt-4 mb-2">Le cœur du rythme :</p>
                      <p className="text-sm text-gray-600 dark:text-gray-400"><code>self.timer</code> : C'est le métronome du script. Il déclenche la fonction principale <code>explore</code> à intervalle régulier.</p>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-3 flex items-center gap-2"><Search className="w-4 h-4" /> 2. <strong><code>find_frontier_points</code> (Le Détecteur de Frontières)</strong></h3>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`def find_frontier_points(self):
    # ...
    # Une frontière est une cellule libre (0) adjacente à une cellule inconnue (-1)
    if grid[y, x] == 0:  # C'est une cellule libre
        # ...
        if grid[y + dy, x + dx] == -1: # Voisin est inconnu
            is_frontier = True
            # ...`}</code></pre></div>
                      <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400 space-y-1">
                          <li><strong>Principe</strong> : La fonction parcourt chaque pixel de la carte.</li>
                          <li><strong>Condition</strong> : Si un pixel est <strong>libre</strong> (valeur 0) ET qu'au moins un de ses 8 voisins est <strong>inconnu</strong> (valeur -1), alors ce pixel est une "frontière".</li>
                          <li><strong>Résultat</strong> : La fonction retourne une liste de toutes les coordonnées `(x, y)` des frontières trouvées.</li>
                      </ul>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-3 flex items-center gap-2"><Target className="w-4 h-4" /> 3. <strong><code>select_best_frontier</code> (Le Stratège)</strong></h3>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`def select_best_frontier(self, frontiers, robot_pose):
    # ...
    # Calcule la distance euclidienne
    dist = math.sqrt((robot_pose.x - wx)**2 + (robot_pose.y - wy)**2)
    if dist < min_dist:
        min_dist = dist
        best_frontier = (wx, wy)
    # ...`}</code></pre></div>
                      <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400 space-y-1">
                          <li><strong>Stratégie</strong> : Choisir la frontière la plus proche du robot.</li>
                          <li><strong>Logique</strong> : Pour chaque frontière, elle convertit les coordonnées, calcule la distance et retient la plus petite.</li>
                          <li><strong>Avantage</strong> : Minimise les longs trajets et encourage une exploration méthodique des zones adjacentes.</li>
                      </ul>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-3 flex items-center gap-2"><Bot className="w-4 h-4" /> 4. <strong><code>explore</code> (Le Chef d'Orchestre)</strong></h3>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`def explore(self):
    # 1. Obtenir la position actuelle du robot
    robot_pose = self.get_robot_pose()
    # 2. Trouver tous les points frontières sur la carte
    frontiers = self.find_frontier_points()
    # ...
    # 3. Sélectionner la meilleure frontière (la plus proche)
    goal_pos = self.select_best_frontier(frontiers, robot_pose)
    # 4. Envoyer le but à Nav2
    # ...`}</code></pre></div>
                      <p className="text-sm text-gray-600 dark:text-gray-400">Le déroulement est une séquence logique simple : <strong>Où suis-je ? -> Où puis-je aller ? -> Quelle est la meilleure option ? -> Allons-y !</strong></p>
                    </div>

                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <Separator className="border-gray-200 dark:border-gray-700" />
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <AlertTriangle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Analyse du Problème : "Pourquoi le robot se coince ?"
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <p className="text-gray-700 dark:text-gray-300">Votre observation est excellente et pointe vers une limitation classique de cette approche simple "fire-and-forget" (tire et oublie).</p>
                    <p className="font-semibold text-gray-800 dark:text-gray-200">Le problème fondamental est que le script n'a pas de "mémoire" de l'objectif en cours ni de retour sur l'état de la navigation de Nav2.</p>
                    <p className="text-gray-700 dark:text-gray-300">Voici le scénario qui se produit :</p>
                    <ol className="list-decimal pl-5 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                      <li><strong>Situation</strong> : Le robot a déjà cartographié une zone. La seule frontière restante est loin.</li>
                      <li><strong>Décision</strong> : Le script trouve cette frontière lointaine et envoie un objectif à Nav2.</li>
                      <li><strong>Navigation</strong> : Nav2 commence son travail, ce qui peut prendre du temps.</li>
                      <li><strong>Le Timer se déclenche à nouveau</strong> : Après 10 secondes, <code>explore</code> est rappelée.</li>
                      <li><strong>Amnésie</strong> : Le script ne sait pas que Nav2 est déjà occupé. Il refait l'analyse et trouve la même frontière.</li>
                      <li><strong>Conflit</strong> : Le script envoie un nouvel objectif au même endroit, ce qui annule et remplace la tâche précédente.</li>
                    </ol>
                    <div className="bg-red-50 dark:bg-red-950/20 border border-red-200 dark:border-red-800 rounded p-3 text-sm">
                      <p className="text-red-800 dark:text-red-300"><strong>Le cas critique (le "blocage") :</strong> Si Nav2 n'arrive pas à atteindre la destination (robot physiquement coincé), il entre en mode de récupération. Pendant ce temps, notre script, ignorant l'échec, continue de lui envoyer le même ordre, le coinçant dans une boucle d'échec sans fin.</p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <GitBranch className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Pistes d'Amélioration (Comment résoudre le problème)
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <p className="text-gray-700 dark:text-gray-300">Pour rendre le script plus intelligent, il doit savoir si Nav2 est "occupé".</p>
                    <div>
                      <h3 className="font-semibold text-gray-800 dark:text-gray-200 mb-2">Solution 1 : Simple (avec un "drapeau")</h3>
                      <p className="text-sm text-gray-600 dark:text-gray-400">On peut ajouter un simple drapeau <code>self.is_navigating</code>. Le problème est de savoir quand le remettre à <code>False</code>, car on ne sait pas quand Nav2 a terminé ou échoué.</p>
                    </div>
                    <div>
                      <h3 className="font-semibold text-gray-800 dark:text-gray-200 mb-2">Solution 2 : Robuste (Utiliser un "Action Client" ROS 2)</h3>
                      <p className="text-gray-700 dark:text-gray-300">La bonne manière de faire avec Nav2 est d'utiliser son <strong>Serveur d'Action</strong>, qui s'appelle <code>/navigate_to_pose</code>. Un "Action" est comme un service mais pour des tâches longues : il donne un retour continu et un résultat final (succès, échec, annulé).</p>
                      <p className="text-gray-700 dark:text-gray-300 mt-2">La logique serait :</p>
                      <ol className="list-decimal pl-5 mt-2 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                        <li>Dans <code>explore</code>, si aucune action n'est en cours : trouver une frontière, envoyer le but via le client d'action, et enregistrer une fonction de "callback".</li>
                        <li>Dans cette fonction de callback (appelée quand l'action se termine) : analyser le résultat (succès/échec) et relancer une nouvelle exploration via le timer.</li>
                      </ol>
                      <p className="mt-4 text-sm bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3">Cette approche est plus complexe à coder mais elle est la seule qui soit vraiment robuste pour interagir avec des systèmes comme Nav2.</p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Download className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Télécharger ce script
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <Button asChild>
                      <Link href="Documentation/semaine-3/IT/Cartographie_Nav2.md">
                        <Download className="w-4 h-4 mr-2" />
                        Télécharger le fichier Cartographie_Nav2.md
                      </Link>
                    </Button>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-3/it/algorithme-pathfinding">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Pathfinding
                  </Button>
                </Link>
                <Link href="/docs/semaine-3/it">
                  <Button>
                    IT Semaine 3
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