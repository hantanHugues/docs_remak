"use client"

import { Button } from "@/components/ui/button"
import { ArrowRight, Trophy, Sparkles } from "lucide-react"
import DotPattern from "@/components/ui/dot-pattern"
import Particles from "@/components/ui/particles"
import { cn } from "@/lib/utils"
import { useTheme } from "next-themes"
import Link from "next/link"

export function HeroSection() {
  const { resolvedTheme } = useTheme()
  const isLightTheme = resolvedTheme === "light"

  return (
    <section className="relative min-h-screen flex items-center justify-center overflow-hidden">
      <DotPattern
        width={20}
        height={20}
        cx={1}
        cy={1}
        cr={1}
        className={cn(
          "absolute inset-0 h-full w-full",
          "[mask-image:radial-gradient(ellipse_at_center,white_40%,transparent_80%)]",
          "dark:fill-slate-700 fill-neutral-300",
        )}
      />
      <Particles
        className="absolute inset-0"
        quantity={150}
        ease={80}
        color={isLightTheme ? "#e91e63" : "#ec4899"}
        refresh
      />

      <div className="absolute inset-0 bg-gradient-to-br from-primary/5 via-transparent to-chart-2/10" />
      <div className="absolute top-0 left-0 w-[500px] h-[500px] bg-primary/20 rounded-full blur-[120px] animate-pulse" />
      <div className="absolute bottom-0 right-0 w-[600px] h-[600px] bg-chart-2/20 rounded-full blur-[120px] animate-pulse [animation-delay:1s]" />

      <div className="container relative z-10 px-4 py-20 mx-auto">
        <div className="max-w-5xl mx-auto text-center space-y-8">
          <div className="inline-flex items-center gap-2 px-5 py-2.5 rounded-full bg-primary/10 border border-primary/30 text-primary text-sm font-semibold shadow-lg shadow-primary/20 backdrop-blur-sm animate-fade-in hover:scale-105 transition-transform">
            <Trophy className="w-4 h-4" />
            <span>TRC 2025 Finalistes • IFRI - Université d'Abomey-Calavi</span>
            <Sparkles className="w-4 h-4" />
          </div>

          <h1 className="text-5xl md:text-7xl lg:text-8xl font-bold tracking-tight text-balance animate-fade-in-up">
            Documentation
            <span className="block bg-gradient-to-r from-primary via-chart-1 to-chart-2 bg-clip-text text-transparent mt-2">
              Technique TRC 2025
            </span>
          </h1>

          {/* Subheading */}
          <p className="text-lg md:text-xl lg:text-2xl text-muted-foreground max-w-3xl mx-auto text-balance leading-relaxed animate-fade-in-up delay-200">
            Découvrez nos solutions techniques, notre méthodologie et nos innovations à travers une documentation détaillée de notre parcours dans la compétition TRC.
          </p>

          <div className="flex flex-col sm:flex-row items-center justify-center gap-4 pt-4 animate-fade-in-up [animation-delay:300ms]">
            <Link href="/pre-selection">
              <Button
                size="lg"
                className="group text-base px-8 py-6 shadow-xl shadow-primary/30 hover:shadow-2xl hover:shadow-primary/40 transition-all hover:scale-105"
              >
                📋 Documentation Pré-sélection
                <ArrowRight className="ml-2 w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Button>
            </Link>
            <Link href="/finale">
              <Button
                size="lg"
                variant="outline"
                className="text-base px-8 py-6 border-2 bg-background/50 backdrop-blur-sm hover:bg-background/80 hover:scale-105 transition-all"
              >
                🚀 Documentation Finale
              </Button>
            </Link>
          </div>

          {/* Elegant features showcase */}
          <div className="pt-16 animate-fade-in-up [animation-delay:500ms]">
            <div className="relative max-w-5xl mx-auto">
              {/* Background decoration */}
              <div className="absolute inset-0 bg-gradient-to-r from-primary/5 via-chart-1/5 to-chart-2/5 rounded-3xl blur-xl" />
              
              <div className="relative bg-background/40 backdrop-blur-xl border border-white/10 rounded-3xl p-8 md:p-12 shadow-2xl">
                <div className="grid md:grid-cols-2 gap-12 items-center">
                  {/* Left side - Features */}
                  <div className="space-y-8">
                    <div className="space-y-6">
                      <div className="flex items-center gap-4 group">
                        <div className="w-12 h-12 rounded-2xl bg-gradient-to-br from-primary/20 to-primary/5 flex items-center justify-center group-hover:scale-110 transition-transform shadow-lg">
                          <div className="w-6 h-6 rounded-lg bg-gradient-to-br from-primary to-chart-1 flex items-center justify-center">
                            <div className="w-2 h-2 bg-white rounded-full" />
                          </div>
                        </div>
                        <div>
                          <h3 className="text-lg font-bold">9 Projets Documentés</h3>
                          <p className="text-sm text-muted-foreground">Solutions techniques détaillées</p>
                        </div>
                      </div>
                      
                      <div className="flex items-center gap-4 group">
                        <div className="w-12 h-12 rounded-2xl bg-gradient-to-br from-chart-1/20 to-chart-1/5 flex items-center justify-center group-hover:scale-110 transition-transform shadow-lg">
                          <div className="w-6 h-6 rounded-lg bg-gradient-to-br from-chart-1 to-chart-2 flex items-center justify-center">
                            <div className="w-2 h-2 bg-white rounded-full" />
                          </div>
                        </div>
                        <div>
                          <h3 className="text-lg font-bold">Code Source Complet</h3>
                          <p className="text-sm text-muted-foreground">Implémentations et algorithmes</p>
                        </div>
                      </div>
                      
                      <div className="flex items-center gap-4 group">
                        <div className="w-12 h-12 rounded-2xl bg-gradient-to-br from-chart-2/20 to-chart-2/5 flex items-center justify-center group-hover:scale-110 transition-transform shadow-lg">
                          <div className="w-6 h-6 rounded-lg bg-gradient-to-br from-chart-2 to-primary flex items-center justify-center">
                            <div className="w-2 h-2 bg-white rounded-full" />
                          </div>
                        </div>
                        <div>
                          <h3 className="text-lg font-bold">Schémas & Modèles</h3>
                          <p className="text-sm text-muted-foreground">Architectures et conceptions</p>
                        </div>
                      </div>
                    </div>
                  </div>
                  
                  {/* Right side - Value proposition */}
                  <div className="space-y-6">
                    <div className="relative">
                      <div className="absolute inset-0 bg-gradient-to-r from-primary/20 to-chart-1/20 rounded-2xl blur-lg" />
                      <div className="relative bg-background/60 backdrop-blur-sm border border-white/10 rounded-2xl p-6">
                        <h3 className="text-xl font-bold mb-4 bg-gradient-to-r from-primary to-chart-1 bg-clip-text text-transparent">
                          Documentation Technique Complète
                        </h3>
                        <div className="space-y-3 text-sm text-muted-foreground">
                          <div className="flex items-center gap-2">
                            <div className="w-1.5 h-1.5 rounded-full bg-primary" />
                            <span>Méthodologies de développement</span>
                          </div>
                          <div className="flex items-center gap-2">
                            <div className="w-1.5 h-1.5 rounded-full bg-chart-1" />
                            <span>Retours d'expérience détaillés</span>
                          </div>
                          <div className="flex items-center gap-2">
                            <div className="w-1.5 h-1.5 rounded-full bg-chart-2" />
                            <span>Analyses techniques approfondies</span>
                          </div>
                          <div className="flex items-center gap-2">
                            <div className="w-1.5 h-1.5 rounded-full bg-primary" />
                            <span>Guides d'implémentation</span>
                          </div>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

    </section>
  )
}
