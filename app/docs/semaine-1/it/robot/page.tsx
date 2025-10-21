"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Bot, Code, List, Settings, Shapes, Puzzle, BookOpen, Target, PlayCircle, HardHat, Network
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';

export default function RobotClassDocFRPage() {
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
                    <Link href="/docs/semaine-1/it" className="hover:text-foreground transition-colors">
                      Documentation Informatique
                    </Link>
                    <span>/</span>
                    <span>Classe : Robot</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Bot className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Classe : <code>Robot</code> (Classe de Base Abstraite)</h1>
                      <p className="text-muted-foreground">Documentation technique de la classe de base pour la simulation de robots.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Python</Badge>
                    <Badge variant="outline">Informatique</Badge>
                    <Badge variant="outline">POO</Badge>
                    <Badge variant="outline">Classe Abstraite</Badge>
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
                      Vue d'ensemble de l'architecture
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Classe abstraite définissant l'interface commune pour tous les robots de simulation
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="prose prose-gray dark:prose-invert max-w-none p-6">
                    <div className="not-prose mb-6">
                      <div className="w-24 h-24 bg-gray-50 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-lg flex items-center justify-center mx-auto">
                        <Image src="/Documentation/semaine-1/it/robot.svg" alt="Icône de Robot" width={48} height={48} className="opacity-60" />
                      </div>
                    </div>
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        La classe <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">Robot</code> constitue le fondement architectural de l'écosystème robotique de simulation. 
                        Cette classe abstraite (ABC - Abstract Base Class) établit le contrat d'interface et encapsule les fonctionnalités essentielles communes à tous les types de robots.
                      </p>
                      <p>
                        <strong>Caractéristiques principales :</strong>
                      </p>
                      <ul className="space-y-1 text-sm">
                        <li>Gestion de l'identité unique et des métadonnées du robot</li>
                        <li>Système de coordonnées et orientation spatiale</li>
                        <li>Modèle énergétique avec sources d'alimentation multiples</li>
                        <li>Interface d'actions fondamentales (mouvement, rotation, arrêt)</li>
                        <li>Architecture extensible pour l'intégration de capteurs</li>
                      </ul>
                      <p className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                        <strong>Note architecturale :</strong> En tant que classe abstraite, <code>Robot</code> ne peut être instanciée directement. 
                        Les implémentations concrètes doivent hériter de cette classe et fournir des implémentations spécifiques pour les méthodes abstraites.
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <List className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Constantes et variables de classe
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Paramètres de configuration globaux et énumérations
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="space-y-6">
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">ENERGY_SOURCE</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Énumération des sources d'énergie supportées par l'architecture robotique.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">List[str]</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Valeurs :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">["solar", "fossil_fuel", "electric"]</code>
                          </div>
                        </div>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">LOW_BATTERY_THRESHOLD</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Seuil critique de niveau énergétique déclenchant les alertes de batterie faible.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">int</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Valeur par défaut :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">20</code> (%)
                          </div>
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
                      <Code className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Constructeur d'initialisation
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Méthode d'instanciation avec validation des paramètres d'entrée
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 my-4 overflow-x-auto relative border border-gray-700">
                      <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                      <pre className="text-sm text-gray-100">
                        <code>{`def __init__(
    self,
    name: str,
    position: Tuple[float, float],
    orientation: float,
    energy_source: str,
) -> None:`}</code>
                      </pre>
                    </div>
                    
                    <div className="space-y-6">
                      <div>
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Paramètres d'entrée
                        </h4>
                        <div className="grid gap-3">
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">name</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Identifiant textuel unique et lisible du robot dans l'environnement de simulation.</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">position</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Coordonnées cartésiennes (x, y) en mètres dans le référentiel global de simulation.</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">orientation</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Orientation angulaire en degrés (conversion automatique en radians).</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">energy_source</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Type de source énergétique (validation contre ENERGY_SOURCE).</span>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>

                      <div>
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Mécanismes de validation
                        </h4>
                        <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-4">
                          <ul className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <li>• Validation de type pour <code>name</code> (string non vide)</li>
                            <li>• Contrôle de format tuple (2 éléments numériques) pour <code>position</code></li>
                            <li>• Vérification de type numérique pour <code>orientation</code></li>
                            <li>• Validation d'appartenance à l'énumération <code>ENERGY_SOURCE</code> (insensible à la casse)</li>
                          </ul>
                        </div>
                      </div>

                      <div>
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Séquence d'initialisation
                        </h4>
                        <div className="space-y-2">
                          {[
                            "Génération d'identifiant UUID4 unique",
                            "Initialisation état inactif (_is_active = False)",
                            "Configuration niveau énergétique maximal (_generator_level = 100)",
                            "Préparation liste de capteurs vide (_sensors = [])",
                            "Configuration du système de journalisation"
                          ].map((step, index) => (
                            <div key={index} className="flex items-start gap-3 text-sm">
                              <span className="w-6 h-6 bg-gray-100 dark:bg-gray-800 rounded-full flex items-center justify-center text-xs font-medium text-gray-600 dark:text-gray-400 flex-shrink-0 mt-0.5">
                                {index + 1}
                              </span>
                              <span className="text-gray-600 dark:text-gray-400">{step}</span>
                            </div>
                          ))}
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
                      <Settings className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Interface des propriétés d'instance
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Accesseurs et mutateurs avec validation intégrée des données
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                      <table className="w-full">
                        <thead className="bg-gray-50 dark:bg-gray-800">
                          <tr>
                            <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Propriété</th>
                            <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Type</th>
                            <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Description fonctionnelle</th>
                          </tr>
                        </thead>
                        <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">id</td>
                            <td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">UUID</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Identifiant unique généré automatiquement (lecture seule)</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">name</td>
                            <td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">str</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Identificateur textuel du robot (lecture seule)</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">position</td>
                            <td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">Tuple[float, float]</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Coordonnées spatiales avec validation de format tuple</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">orientation</td>
                            <td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">float</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Orientation angulaire normalisée sur [0, 2π) radians</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">energy_source</td>
                            <td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">str</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Type de source énergétique (immutable après initialisation)</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">generator_level</td>
                            <td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">int</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Niveau énergétique avec contrainte d'intervalle [0, 100]</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">is_active</td>
                            <td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">bool</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">État opérationnel avec dépendance énergétique</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">sensors</td>
                            <td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">List[Any]</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Collection de capteurs avec gestion d'ajout/suppression</td>
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
                      <Shapes className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Méthodes d'instance concrètes
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Implémentations fournies par la classe de base
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-6 p-6">
                    <div className="space-y-8">
                      {/* Méthode start */}
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">start(self) → None</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Procédure d'activation conditionnelle du robot avec contrôle énergétique préalable.
                        </p>
                        <ul className="text-sm text-gray-600 dark:text-gray-400 space-y-1 mb-4">
                          <li>• Vérification du niveau énergétique (generator_level &gt; 0)</li>
                          <li>• Activation du robot si conditions respectées</li>
                          <li>• Journalisation des événements d'activation et d'échec</li>
                        </ul>
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                          <pre className="text-sm text-gray-100"><code>{`def start(self) -> None:
    """
    Démarre les opérations du robot.
    """
    if self._generator_level <= 0:
        self._logger.warning("Démarrage impossible : Le niveau du générateur est à zéro.")
        return
    self.is_active = True
    self._logger.info(f"Robot {self.name} démarré.")`}</code></pre>
                        </div>
                      </div>
                      
                      <Separator className="border-gray-200 dark:border-gray-700" />
                      
                      {/* Méthode distance_to */}
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">distance_to(self, other: Tuple[float, float]) → float</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Calcul de distance euclidienne entre la position actuelle du robot et un point de référence.
                        </p>
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                          <pre className="text-sm text-gray-100"><code>{`def distance_to(self, other: Tuple[float, float]) -> float:
    """
    Calcule la distance entre le robot et un autre point.

    Arguments:
        other (Tuple[float, float]): Les coordonnées (x, y) de l'autre point.

    Retourne:
        float: La distance jusqu'à l'autre point.
    """
    x0, y0 = self.position
    x1, y1 = other
    return math.hypot(x1 - x0, y1 - y0)`}</code></pre>
                        </div>
                      </div>

                      <Separator className="border-gray-200 dark:border-gray-700" />

                      {/* Méthode consume_energy */}
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">consume_energy(self, amount: float) → None</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Gestion de la consommation énergétique avec validation et journalisation.
                        </p>
                        <ul className="text-sm text-gray-600 dark:text-gray-400 space-y-1 mb-4">
                          <li>• Contrôle de l'état actif du robot avant consommation</li>
                          <li>• Validation de la valeur positive du paramètre amount</li>
                          <li>• Décrément du niveau énergétique et journalisation</li>
                        </ul>
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                          <pre className="text-sm text-gray-100"><code>{`def consume_energy(self, amount: float) -> None:
    """Consomme de l'énergie de la batterie du robot."""
    if not self._is_active:
        self._logger.warning("Le robot est inactif, impossible de consommer de l'énergie.")
        return
    if amount < 0:
        raise ValueError("La consommation d'énergie doit être positive.")
    self._generator_level -= amount
    self._logger.info(f"{amount} d'énergie consommée. Niveau du générateur est maintenant de {self._generator_level}.")`}</code></pre>
                        </div>
                      </div>
                      
                      <Separator className="border-gray-200 dark:border-gray-700" />

                      {/* Méthodes de gestion des capteurs */}
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">add_sensor / remove_sensor</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Interface de gestion de la collection de capteurs avec prévention des doublons.
                        </p>
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                          <pre className="text-sm text-gray-100"><code>{`def add_sensor(self, sensor: Any) -> None:
    """
    Ajoute un capteur au robot.
    
    Arguments:
        sensor (Any): Le capteur à ajouter.
    """
    if sensor not in self._sensors:
        self._sensors.append(sensor)
        self._logger.info(f"Capteur {sensor} ajouté au robot {self.name}.")
    else:
        self._logger.warning(f"Le capteur {sensor} est déjà attaché au robot {self.name}.")

def remove_sensor(self, sensor: Any) -> None:
    """
    Retire un capteur du robot.
    
    Arguments:
        sensor (Any): Le capteur à retirer.
    """
    if sensor in self._sensors:
        self._sensors.remove(sensor)
        self._logger.info(f"Capteur {sensor} retiré du robot {self.name}.")
    else:
        self._logger.warning(f"Le capteur {sensor} n'a pas été trouvé sur le robot {self.name}.")`}</code></pre>
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
                      <Puzzle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Contrat d'interface abstraite
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Méthodes obligatoires à implémenter dans les classes dérivées
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="space-y-6">
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">
                          <strong>Principe d'abstraction :</strong> Ces méthodes définissent le comportement minimal que toute implémentation 
                          concrète de robot doit fournir. L'absence d'implémentation génère une <code>NotImplementedError</code> 
                          à l'exécution.
                        </p>
                      </div>

                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 my-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`@abstractmethod
def move(self, direction: str, distance: float) -> None:
    """
    Déplace le robot dans une direction spécifiée sur une certaine distance.
    
    Arguments:
        direction (str): La direction du déplacement ('forward', 'backward').
        distance (float): La distance à parcourir dans la direction spécifiée.
    """
    raise NotImplementedError

@abstractmethod
def rotate(self, angle: float) -> None:
    """
    Fait pivoter le robot d'un certain angle.

    Arguments:
        angle (float): L'angle de rotation du robot, en radians.
    """
    raise NotImplementedError

@abstractmethod
def stop(self) -> None:
    """
    Arrête le mouvement du robot.
    """
    raise NotImplementedError

@abstractmethod
def status(self) -> str:
    """
    Obtient le statut actuel du robot.
    
    Retourne:
        str: Une chaîne de caractères représentant le statut actuel du robot.
    """
    return (f"Robot(ID: {self.id}, Nom: {self.name}, Position: {self.position}, "
            f"Orientation: {self.orientation}, Source d'énergie: {self.energy_source}, "
            f"Actif: {self.is_active}), Niveau du générateur: {self.generator_level}")`}</code></pre>
                      </div>
                      
                      <div className="grid gap-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Signatures des méthodes abstraites
                        </h4>
                        <div className="space-y-3">
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm">move(self, direction: str, distance: float) → None</code>
                            <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Interface de déplacement directionnel avec paramètres de distance</p>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm">rotate(self, angle: float) → None</code>
                            <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Méthode de rotation angulaire en radians</p>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm">stop(self) → None</code>
                            <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Arrêt d'urgence et cessation de mouvement</p>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm">status(self) → str</code>
                            <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Génération de rapport d'état compresensif</p>
                          </div>
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
                      <Puzzle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Protocoles Python intégrés
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Surcharges des méthodes spéciales (dunder methods) pour l'intégration système
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="space-y-6">
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          <strong>Intégration avec l'écosystème Python :</strong> Ces méthodes permettent aux instances de Robot 
                          de s'intégrer naturellement avec les fonctions builtin de Python (print, repr, comparaisons).
                        </p>
                      </div>

                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 my-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def __str__(self) -> str:
    """
    Retourne une représentation en chaîne de caractères du robot.
    
    Retourne:
        str: Une chaîne contenant l'ID, le nom, la position, l'orientation, la source d'énergie et le statut actif du robot.
    """
    return self.status()

def __repr__(self) -> str:
    """
    Retourne une représentation détaillée en chaîne de caractères du robot pour le débogage.
    
    Retourne:
        str: Une chaîne contenant le nom de la classe et les attributs du robot.
    """
    return (f"{self.__class__.__name__}(ID={self.id}, Nom={self.name}, "
            f"Position={self.position}, Orientation={self.orientation}, "
            f"Source d'énergie={self.energy_source}, Actif={self.is_active})")

def __eq__(self, other: object) -> bool:
    """
    Vérifie si deux robots sont égaux en se basant sur leur ID.
    
    Arguments:
        other (object): L'objet à comparer.
    
    Retourne:
        bool: True si les ID sont identiques, False sinon.
    """
    if not isinstance(other, Robot):
        raise TypeError("La comparaison n'est supportée qu'entre des instances de Robot.")
    return self.id == other.id`}</code></pre>
                      </div>
                      
                      <div className="grid gap-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Protocoles implémentés
                        </h4>
                        <div className="space-y-4">
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-4">
                            <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-2">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">__str__(self) → str</code>
                            </h5>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Représentation textuelle utilisateur-friendly via délégation à <code>status()</code>. 
                              Utilisée par <code>print()</code> et <code>str()</code>.
                            </p>
                          </div>
                          
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-4">
                            <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-2">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">__repr__(self) → str</code>
                            </h5>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Représentation développeur avec nom de classe et attributs principaux. 
                              Optimisée pour le débogage et la journalisation technique.
                            </p>
                          </div>
                          
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-4">
                            <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-2">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">__eq__(self, other: object) → bool</code>
                            </h5>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Égalité basée sur l'identifiant UUID unique avec validation de type stricte. 
                              Lève <code>TypeError</code> pour les comparaisons inter-types.
                            </p>
                          </div>
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
                      <Network className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Modélisation UML de l'architecture
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Diagramme de classe illustrant la structure abstraite et les relations d'héritage
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <Image src="/Documentation/semaine-1/it/diagram.png" alt="Diagramme UML de la classe Robot" width={800} height={600} className="rounded-md mx-auto w-full object-contain" />
                    </div>
                    <div className="mt-4 text-sm text-gray-600 dark:text-gray-400">
                      <p>
                        <strong>Notation UML :</strong> Ce diagramme présente la classe abstraite Robot avec ses attributs privés, 
                        propriétés publiques, méthodes concrètes et contrat d'interface abstraite. Les relations d'héritage 
                        vers les classes concrètes sont représentées par les flèches de généralisation.
                      </p>
                    </div>
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