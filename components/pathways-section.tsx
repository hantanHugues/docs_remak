import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Flame, Trophy, ArrowRight, CheckCircle2, Sparkles } from "lucide-react"
import Link from "next/link"

export function PathwaysSection() {
  return (
    <section className="relative py-24 px-4 overflow-hidden">
      <div className="absolute inset-0 bg-gradient-to-b from-background via-secondary/10 to-background" />
      <div className="absolute top-1/4 left-0 w-96 h-96 bg-chart-1/10 rounded-full blur-[120px] animate-pulse" />
      <div className="absolute bottom-1/4 right-0 w-96 h-96 bg-primary/10 rounded-full blur-[120px] animate-pulse [animation-delay:1s]" />

      <div className="container relative mx-auto max-w-7xl">
        <div className="text-center space-y-4 mb-16">
          <div className="inline-flex items-center justify-center gap-2 mb-4">
            <Sparkles className="w-5 h-5 text-primary" />
            <span className="text-sm font-semibold text-primary uppercase tracking-wider">Notre Parcours</span>
          </div>
          <h2 className="text-4xl md:text-5xl lg:text-6xl font-bold text-balance">
            Deux Parcours, Un{" "}
            <span className="bg-gradient-to-r from-primary via-chart-1 to-chart-2 bg-clip-text text-transparent">
              Objectif
            </span>
          </h2>
          <p className="text-lg md:text-xl text-muted-foreground max-w-2xl mx-auto text-balance leading-relaxed">
            De la formation intensive à la réalisation d'un système robotique complet
          </p>
        </div>

        <div className="grid lg:grid-cols-2 gap-10">
          {/* Pre-selection Card */}
          <Card className="group relative hover:shadow-2xl transition-all duration-500 border-2 hover:border-chart-1/50 overflow-hidden bg-gradient-to-br from-card to-card/50 backdrop-blur-sm hover:-translate-y-2">
            <div className="h-3 bg-gradient-to-r from-chart-1 via-chart-5 to-chart-3" />
            <div className="absolute inset-0 bg-gradient-to-br from-chart-1/5 to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-500" />
            <CardHeader className="relative space-y-4 pb-6">
              <div className="flex items-center gap-4">
                <div className="w-14 h-14 rounded-2xl bg-gradient-to-br from-chart-1/30 to-chart-1/10 flex items-center justify-center text-chart-1 shadow-lg group-hover:scale-110 group-hover:rotate-6 transition-all duration-500">
                  <Flame className="w-7 h-7" />
                </div>
                <CardTitle className="text-3xl md:text-4xl">Pré-sélection</CardTitle>
              </div>
              <p className="text-xl font-semibold text-chart-1">Le chemin vers l'excellence</p>
            </CardHeader>
            <CardContent className="relative space-y-6">
              <p className="text-muted-foreground leading-relaxed">
                3 semaines intenses de projets techniques pour maîtriser les fondamentaux de la robotique à travers 9
                défis progressifs.
              </p>

              <div className="space-y-3">
                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-chart-1 mt-0.5 flex-shrink-0" />
                  <div>
                    <div className="font-semibold">Semaine 1-3</div>
                    <div className="text-sm text-muted-foreground">Montée en compétences progressive</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-chart-1 mt-0.5 flex-shrink-0" />
                  <div>
                    <div className="font-semibold">9 Défis Techniques</div>
                    <div className="text-sm text-muted-foreground">3 projets par domaine d'expertise</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-chart-1 mt-0.5 flex-shrink-0" />
                  <div>
                    <div className="font-semibold">Méthodologie Rigoureuse</div>
                    <div className="text-sm text-muted-foreground">Apprentissage des bonnes pratiques</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-chart-1 mt-0.5 flex-shrink-0" />
                  <div>
                    <div className="font-semibold">Test Final</div>
                    <div className="text-sm text-muted-foreground">Validation des acquis techniques</div>
                  </div>
                </div>
              </div>

              <Link href="/pre-selection" className="block">
                <Button className="w-full group mt-4 shadow-lg hover:shadow-xl transition-all" size="lg">
                  Découvrir la Pré-sélection
                  <ArrowRight className="ml-2 w-4 h-4 group-hover:translate-x-1 transition-transform" />
                </Button>
              </Link>
            </CardContent>
          </Card>

          {/* Finals Card */}
          <Card className="group relative hover:shadow-2xl transition-all duration-500 border-2 hover:border-primary/50 overflow-hidden bg-gradient-to-br from-card to-card/50 backdrop-blur-sm hover:-translate-y-2">
            <div className="h-3 bg-gradient-to-r from-primary via-chart-1 to-chart-2" />
            <div className="absolute inset-0 bg-gradient-to-br from-primary/5 to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-500" />
            <CardHeader className="relative space-y-4 pb-6">
              <div className="flex items-center gap-4">
                <div className="w-14 h-14 rounded-2xl bg-gradient-to-br from-primary/30 to-primary/10 flex items-center justify-center text-primary shadow-lg group-hover:scale-110 group-hover:rotate-6 transition-all duration-500">
                  <Trophy className="w-7 h-7" />
                </div>
                <CardTitle className="text-3xl md:text-4xl">Finale</CardTitle>
              </div>
              <p className="text-xl font-semibold text-primary">L'aboutissement du projet</p>
            </CardHeader>
            <CardContent className="relative space-y-6">
              <p className="text-muted-foreground leading-relaxed">
                Un système robotique complet et intégré démontrant l'excellence technique et l'innovation.
              </p>

              <div className="space-y-3">
                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-primary mt-0.5 flex-shrink-0" />
                  <div>
                    <div className="font-semibold">DofBot</div>
                    <div className="text-sm text-muted-foreground">Bras robotique de manipulation précise</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-primary mt-0.5 flex-shrink-0" />
                  <div>
                    <div className="font-semibold">Jetson Nano</div>
                    <div className="text-sm text-muted-foreground">Vision par ordinateur intelligente</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-primary mt-0.5 flex-shrink-0" />
                  <div>
                    <div className="font-semibold">ROS Master</div>
                    <div className="text-sm text-muted-foreground">Coordination et orchestration système</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <CheckCircle2 className="w-5 h-5 text-primary mt-0.5 flex-shrink-0" />
                  <div>
                    <div className="font-semibold">Convoyeur + Interface Web</div>
                    <div className="text-sm text-muted-foreground">Transport automatisé et contrôle</div>
                  </div>
                </div>
              </div>

              <Link href="/finale" className="block">
                <Button className="w-full group mt-4 shadow-lg hover:shadow-xl transition-all" size="lg">
                  Explorer le Projet Final
                  <ArrowRight className="ml-2 w-4 h-4 group-hover:translate-x-1 transition-transform" />
                </Button>
              </Link>
            </CardContent>
          </Card>
        </div>

        <div className="mt-20 p-10 rounded-3xl bg-gradient-to-r from-accent/20 via-primary/10 to-chart-2/20 border-2 border-primary/30 backdrop-blur-sm shadow-xl relative overflow-hidden">
          <div className="absolute top-0 left-1/4 w-64 h-64 bg-primary/20 rounded-full blur-[80px]" />
          <div className="absolute bottom-0 right-1/4 w-64 h-64 bg-chart-2/20 rounded-full blur-[80px]" />
          <div className="relative">
            <h3 className="text-3xl font-bold text-center mb-12">Notre Parcours</h3>
            <div className="flex flex-col md:flex-row items-center justify-between gap-6 max-w-5xl mx-auto">
              {[
                { num: 1, title: "Apprentissage", desc: "Bases techniques" },
                { num: 2, title: "Maîtrise", desc: "Projets techniques" },
                { num: 3, title: "Innovation", desc: "Intégration système" },
                { num: 4, title: "Excellence", desc: "Démonstration finale" },
              ].map((step, idx) => (
                <>
                  <div key={step.num} className="text-center space-y-3 group">
                    <div className="w-20 h-20 rounded-2xl bg-gradient-to-br from-primary to-chart-1 text-primary-foreground flex items-center justify-center text-2xl font-bold mx-auto shadow-xl group-hover:scale-110 group-hover:rotate-6 transition-all duration-500">
                      {step.num}
                    </div>
                    <div className="font-bold text-lg">{step.title}</div>
                    <div className="text-sm text-muted-foreground">{step.desc}</div>
                  </div>
                  {idx < 3 && <ArrowRight className="w-8 h-8 text-primary rotate-90 md:rotate-0 flex-shrink-0" />}
                </>
              ))}
            </div>
          </div>
        </div>
      </div>
    </section>
  )
}
