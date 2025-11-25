"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Bot, Code, List, Settings, Shapes, Puzzle, BookOpen, Target, PlayCircle, HardHat
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';


export default function RoboticArmDocFRPage() {
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
                    <span>Classe : RoboticArm</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <HardHat className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Documentation de la Classe <code>RoboticArm</code></h1>
                      <p className="text-muted-foreground">Implémentation concrète d'un bras robotique articulé.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Python</Badge>
                    <Badge variant="outline">Informatique</Badge>
                    <Badge variant="outline">POO</Badge>
                    <Badge variant="outline">Classe Concrète</Badge>
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
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Architecture de classe concrète
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Implémentation spécialisée héritant de la classe abstraite Robot
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="pt-6 prose prose-gray dark:prose-invert max-w-none p-6">
                    <div className="not-prose mb-6">
                      <div className="w-24 h-24 bg-gray-50 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-lg flex items-center justify-center mx-auto">
                        <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/it/robotic_arm.svg" alt="Bras Robotique" width={48} height={48} className="opacity-60" />
                      </div>
                    </div>
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        La classe <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">RoboticArm</code> constitue une implémentation 
                        concrète de la classe de base abstraite <code>Robot</code>, spécialisée dans la modélisation d'un système 
                        robotique articulé à plusieurs degrés de liberté.
                      </p>
                      <p>
                        <strong>Caractéristiques spécialisées :</strong>
                      </p>
                      <ul className="space-y-1 text-sm">
                        <li>Architecture articulée avec contrôle précis des angles</li>
                        <li>Cinématique inverse simplifiée pour positionnement spatial</li>
                        <li>Système de manipulation d'objets avec effecteur terminal</li>
                        <li>Gestion d'état pour objets saisis et relâchés</li>
                        <li>Interface de contrôle adaptée aux applications industrielles</li>
                      </ul>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={50}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Target className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Objectifs fonctionnels
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Spécifications techniques et cas d'usage du système robotique
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="prose prose-gray dark:prose-invert max-w-none p-6">
                    <div className="space-y-4">
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Simulation de système articulé</h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Modélisation d'un bras robotique stationnaire ou monté avec plusieurs degrés de liberté 
                          représentés par des articulations configurables.
                        </p>
                      </div>
                      
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Contrôle de mouvement précis</h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Interface de contrôle pour les angles articulaires, positionnement spatial de l'effecteur terminal 
                          et opérations de manipulation d'objets.
                        </p>
                      </div>
                      
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Gestion d'état avancée</h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Surveillance continue de l'état du bras incluant la position, l'orientation et la gestion 
                          des objets manipulés par l'effecteur terminal.
                        </p>
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
                      Propriétés d'instance spécialisées
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Attributs étendus héritant de la classe Robot avec spécialisations robotiques
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="prose prose-gray dark:prose-invert max-w-none p-6">
                    <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4 mb-6">
                      <p className="text-sm text-gray-600 dark:text-gray-400 mb-0">
                        <strong>Héritage :</strong> Toutes les propriétés de la classe <code>Robot</code> sont héritées, 
                        incluant <code>id</code>, <code>name</code>, <code>position</code> (base), <code>orientation</code> (base), 
                        <code>energy_source</code>, <code>generator_level</code>, <code>is_active</code>, et <code>sensors</code>.
                      </p>
                    </div>
                    
                    <div className="space-y-6">
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">joint_angles</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Vecteur des angles articulaires courants pour chaque degré de liberté du bras robotique.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">List[float]</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Unité :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Degrés [0, 360)</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture-écriture avec normalisation</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Longueur :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Déterminée par num_joints</span>
                          </div>
                        </div>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">num_joints</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Nombre d'articulations configurant la complexité cinématique du bras robotique.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">int</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Contrainte :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Minimum 2 articulations</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture seule (immutable)</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Défaut :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">6 articulations</span>
                          </div>
                        </div>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">end_effector_position</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Coordonnées spatiales actuelles de l'effecteur terminal (pince/outil) dans l'espace de travail.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">Tuple[float, float]</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Référentiel :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Coordonnées (x, y) absolues</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture seule (calculé)</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Mise à jour :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Via méthode move()</span>
                          </div>
                        </div>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">holding_item</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Référence vers l'objet actuellement saisi par l'effecteur terminal.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">Optional[str]</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">État initial :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">None (aucun objet)</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture seule (contrôlé)</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Gestion :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">pick() et place()</span>
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
                      <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">__init__(name, position, orientation, energy_source, num_joints=6)</code>
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="space-y-6">
                      <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                        <p className="mb-4">
                          Procédure d'instanciation d'un nouveau bras robotique avec configuration des paramètres 
                          cinématiques et énergétiques spécialisés.
                        </p>
                      </div>

                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 my-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100">
                          <code>{`def __init__(
        self,
        name: str,
        position: Tuple[float, float],
        orientation: float,
        energy_source: str,
        num_joints: int = 6
) -> None:
    """
    Initialise le bras robotique avec les paramètres donnés.
    """
    super().__init__(name=name, position=position, orientation=orientation, energy_source=energy_source)
    if not isinstance(num_joints, int) or num_joints < 2:
        raise ValueError("Un bras robotique doit avoir au moins 2 articulations.")
    self._joint_angles: List[float] = [0.0] * num_joints  # Angles en degrés
    self._num_joints = num_joints
    self._end_effector_position: Tuple[float, float] = position
    self._holding_item: Optional[str] = None`}</code>
                        </pre>
                      </div>

                      <div>
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Paramètres d'entrée
                        </h4>
                        <div className="space-y-3">
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">name</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Identifiant textuel unique du bras robotique.</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">position</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Coordonnées (x, y) de la base du bras dans le référentiel global.</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">orientation</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Orientation de la base du bras en radians.</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">energy_source</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Source d'énergie validée contre l'énumération Robot.ENERGY_SOURCE.</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">num_joints</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Nombre d'articulations pour le bras. Valeur par défaut : 6. Doit être &gt;= 2.</span>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>

                      <div>
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Exceptions levées
                        </h4>
                        <div className="space-y-2">
                          <div className="bg-red-50 dark:bg-red-950/20 border border-red-200 dark:border-red-800 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-red-100 dark:bg-red-900 px-2 py-1 rounded text-sm font-mono">ValueError</code>
                              <span className="text-sm text-gray-600 dark:text-gray-400">Si num_joints &lt; 2 ou energy_source invalide.</span>
                            </div>
                          </div>
                          <div className="bg-red-50 dark:bg-red-950/20 border border-red-200 dark:border-red-800 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-red-100 dark:bg-red-900 px-2 py-1 rounded text-sm font-mono">TypeError</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Pour des types de paramètres invalides (validation parent ou directe).</span>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>

                      <div>
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Exemple d'utilisation
                        </h4>
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mt-4 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                          <pre className="text-sm text-gray-100"><code>{`from robot import Robot
from roboticArm import RoboticArm
import math

arm = RoboticArm(
    name="Dexter",
    position=(0.0, 0.0),
    orientation=0.0,
    energy_source="electric",
    num_joints=4
)
print(f"Bras robotique créé : {arm.name} avec {arm.num_joints} articulations.")
print(f"Angles initiaux des articulations : {arm.joint_angles}")`}</code></pre>
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
                      <List className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Résumé des propriétés et accesseurs
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Interface publique des propriétés avec comportements associés
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                      <table className="w-full">
                        <thead className="bg-gray-50 dark:bg-gray-800">
                          <tr>
                            <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Propriété</th>
                            <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Comportement fonctionnel</th>
                          </tr>
                        </thead>
                        <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">joint_angles</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Copie des angles actuels en degrés (lecture/écriture avec normalisation)</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">num_joints</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Nombre d'articulations configurées (lecture seule, immutable)</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">end_effector_position</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Position (x, y) calculée de l'effecteur terminal (lecture seule)</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">holding_item</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Référence d'objet saisi ou None (lecture seule, contrôlé par pick/place)</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">joint_angles (setter)</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Validation de longueur, types numériques et normalisation [0, 360)</td>
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
                      Interface des méthodes d'instance
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Implémentations des méthodes abstraites et fonctionnalités spécialisées
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    {/* Method: move */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">move(target_position)</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Déplace l'effecteur terminal du bras à une position cible (x, y) spécifiée. Il s'agit d'un mouvement 2D simplifié.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def move(self, target_position: Tuple[float, float]) -> None:
    """
    Déplace l'effecteur terminal à une position cible (2D pour simplifier).
    """
    if not self.is_active:
        self._logger.warning("Déplacement impossible : le bras robotique est inactif.")
        return
    if not (isinstance(target_position, tuple) and len(target_position) == 2):
        raise ValueError("La position cible doit être un tuple (x, y).")
    
    angle = math.degrees(math.atan2(target_position[1] - self.position[1], target_position[0] - self.position[0]))
    self._joint_angles = [angle / self._num_joints] * self._num_joints
    self._end_effector_position = target_position
    self.consume_energy(1.0)
    self._logger.info(f"Effecteur terminal déplacé à {target_position} avec les angles d'articulation {self._joint_angles}")`}</code></pre>
                      </div>

                      <div className="space-y-3">
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                          <p className="text-sm text-gray-900 dark:text-gray-100">
                            <strong>Paramètres :</strong> <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">target_position</code> (<code>Tuple[float, float]</code>) - Les coordonnées (x, y) désirées pour l'effecteur terminal.
                          </p>
                        </div>
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                          <p className="text-sm text-gray-900 dark:text-gray-100">
                            <strong>Description :</strong> Si actif, calcule un angle simplifié vers la cible et le répartit entre les articulations. Met à jour <code>_end_effector_position</code>. Consomme de l'énergie. Journalise le mouvement.
                          </p>
                        </div>
                        <div className="bg-red-50 dark:bg-red-950/20 border border-red-200 dark:border-red-800 rounded p-3">
                          <p className="text-sm text-gray-900 dark:text-gray-100">
                            <strong>Lève :</strong> <code className="bg-red-100 dark:bg-red-900 px-1 rounded">ValueError</code> si <code>target_position</code> n'est pas un tuple (x, y).
                          </p>
                        </div>
                      </div>

                      <div>
                        <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Exemple d'utilisation</h5>
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                          <pre className="text-sm text-gray-100"><code>{`arm.start()
arm.generator_level = 100
print(f"Position initiale de l'effecteur terminal : {arm.end_effector_position}")
arm.move((1.0, 0.5)) # Déplacement vers (1.0, 0.5)
print(f"Nouvelle position de l'effecteur terminal : {arm.end_effector_position}, Angles des articulations : {arm.joint_angles}")`}</code></pre>
                        </div>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />

                    {/* Method: rotate */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">rotate(joint_index, angle)</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Fait pivoter une articulation spécifique du bras d'un angle donné (rotation relative).
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def rotate(self, joint_index: int, angle: float) -> None:
    """
    Fait pivoter une articulation spécifique d'un angle donné (en degrés).
    """
    if not self.is_active:
        self._logger.warning("Rotation impossible : le bras robotique est inactif.")
        return
    if not (0 <= joint_index < self._num_joints):
        raise IndexError("Index d'articulation invalide.")
    
    self._joint_angles[joint_index] = (self._joint_angles[joint_index] + angle) % 360
    self.orientation = (math.degrees(self.orientation) + angle) % 360
    self.consume_energy(0.1)
    self._logger.info(f"Articulation {joint_index} pivotée de {angle} degrés. Nouvel angle : {self._joint_angles[joint_index]}")`}</code></pre>
                      </div>

                      <div className="space-y-3">
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                          <p className="text-sm text-gray-900 dark:text-gray-100">
                            <strong>Paramètres :</strong> <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">joint_index</code> (<code>int</code>) - L'index de l'articulation à faire pivoter (de 0 à <code>num_joints - 1</code>). <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">angle</code> (<code>float</code>) - L'angle en degrés pour la rotation (peut être positif ou négatif).
                          </p>
                        </div>
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                          <p className="text-sm text-gray-900 dark:text-gray-100">
                            <strong>Description :</strong> Si actif, met à jour l'angle de l'articulation spécifiée (normalisé à 0-360). Met également à jour l'<code>orientation</code> de la base du bras du même angle. Consomme de l'énergie. Journalise la rotation.
                          </p>
                        </div>
                        <div className="bg-red-50 dark:bg-red-950/20 border border-red-200 dark:border-red-800 rounded p-3">
                          <p className="text-sm text-gray-900 dark:text-gray-100">
                            <strong>Lève :</strong> <code className="bg-red-100 dark:bg-red-900 px-1 rounded">IndexError</code> si <code>joint_index</code> est invalide.
                          </p>
                        </div>
                      </div>

                      <div>
                        <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Exemple d'utilisation</h5>
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                          <pre className="text-sm text-gray-100"><code>{`arm.start()
arm.generator_level = 100
print(f"Angle initial de l'articulation 0 : {arm.joint_angles[0]}")
arm.rotate(joint_index=0, angle=45.0)
print(f"Nouvel angle de l'articulation 0 : {arm.joint_angles[0]}, Orientation du bras : {arm.orientation} rad")`}</code></pre>
                        </div>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: stop */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">stop()</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Arrête tous les mouvements du bras robotique et le désactive.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def stop(self) -> None:
    """
    Arrête tous les mouvements du bras robotique.
    """
    self.is_active = False
    self._logger.info("Bras robotique arrêté.")`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: status */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">status()</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Fournit une chaîne de caractères résumant le statut actuel du <code>RoboticArm</code>.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def status(self) -> str:
    return (f"RoboticArm(ID: {self.id}, Nom: {self.name}, Position: {self.position}, "
            f"Effecteur-Terminal: {self._end_effector_position}, Orientation: {self.orientation:.2f} rad, "
            f"Articulations: {self._joint_angles}, Tient: {self._holding_item}, "
            f"Actif: {self.is_active}, Niveau du générateur: {self.generator_level})")`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />

                    {/* Method: set_joint_angle */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">set_joint_angle(joint_index, angle)</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Règle une articulation spécifique à un angle absolu donné.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def set_joint_angle(self, joint_index: int, angle: float) -> None:
    """
    Règle une articulation spécifique à un angle donné (en degrés).
    """
    if not (0 <= joint_index < self._num_joints):
        raise IndexError("Index d'articulation invalide.")
    self._joint_angles[joint_index] = angle % 360
    self._logger.info(f"Articulation {joint_index} réglée à {angle} degrés.")`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />

                    {/* Method: reset_arm */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">reset_arm()</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Réinitialise tous les angles d'articulation à zéro, déplace l'effecteur terminal à la position de base du bras et libère tout objet tenu.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def reset_arm(self) -> None:
    self._joint_angles = [0.0] * self._num_joints
    self._end_effector_position = self.position
    self._holding_item = None
    self._logger.info("Bras robotique réinitialisé à la position d'origine.")`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: pick */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">pick(item)</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Saisit un objet avec l'effecteur terminal du bras robotique.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def pick(self, item: str) -> None:
    """
    Saisit un objet avec l'effecteur terminal du bras robotique.
    """
    if not self.is_active:
        self._logger.warning(f"Impossible de saisir {item}: le bras robotique est inactif.")
        return
    if self._holding_item is not None:
        self._logger.warning(f"Tient déjà {self._holding_item}, impossible de saisir {item}.")
        return
    self._holding_item = item
    self.consume_energy(0.5)
    self._logger.info(f"A saisi {item} à {self._end_effector_position}.")`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />

                    {/* Method: place */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">place(item, target_position)</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Place l'objet actuellement tenu à une position cible.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def place(self, item: str, target_position: Tuple[float, float]) -> None:
    """
    Place l'objet actuellement tenu à une position cible ou dans un sac de stockage.
    """
    if not self.is_active:
        self._logger.warning(f"Impossible de placer {item}: le bras robotique est inactif.")
        return
    if self._holding_item != item:
        self._logger.warning(f"Impossible de placer {item}: ne le tient pas actuellement.")
        return
    self.move(target_position)
    self._holding_item = None
    self.consume_energy(0.5)
    self._logger.info(f"A placé {item} à {target_position}.")`}</code></pre>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Scénario d'Exemple : Opération de Prise et de Placement
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Démonstration complète des fonctionnalités du bras robotique
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-600 dark:text-gray-400 mb-4">
                      Ce scénario démontre l'initialisation d'un <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">RoboticArm</code>, son activation, le déplacement de son effecteur terminal, la prise d'un objet, un nouveau déplacement, puis le placement de l'objet.
                    </p>
                    <div className="bg-blue-50 dark:bg-blue-950/20 border border-blue-200 dark:border-blue-800 rounded p-4">
                      <p className="text-sm text-gray-900 dark:text-gray-100">
                        <strong>Ressource additionnelle :</strong> Consultez les tests unitaires pour des exemples d'utilisation détaillés dans <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">test_robotic_arm.py</code>
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Navigation footer */}
              <div className="flex items-center justify-between pt-8 border-t border-gray-200 dark:border-gray-700">
                <Link href="../robot">
                  <Button variant="outline" className="text-gray-600 dark:text-gray-400 border-gray-300 dark:border-gray-600 hover:bg-gray-50 dark:hover:bg-gray-800">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Classe Robot
                  </Button>
                </Link>
                <Link href="#">
                  <Button className="bg-gray-900 dark:bg-gray-100 text-white dark:text-gray-900 hover:bg-gray-800 dark:hover:bg-gray-200">
                    Module Suivant
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