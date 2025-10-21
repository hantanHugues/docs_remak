import { Button } from "@/components/ui/button"
import { ArrowRight, Flame, Trophy } from "lucide-react"
import Link from "next/link"

export function CtaSection() {
  return (
    <section className="relative py-32 px-4 overflow-hidden">
      <div className="absolute inset-0 bg-gradient-to-b from-background via-primary/5 to-background" />
      <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[1000px] h-[600px] bg-gradient-to-r from-primary/20 to-chart-1/20 rounded-full blur-[150px] animate-pulse" />

      <div className="container relative mx-auto max-w-5xl">
        <div className="text-center space-y-8">
          <div className="inline-flex items-center justify-center gap-2 mb-4">
            <Flame className="w-6 h-6 text-primary" />
            <span className="text-sm font-semibold text-primary uppercase tracking-wider">Prêt à Découvrir</span>
          </div>

          <h2 className="text-4xl md:text-5xl lg:text-6xl font-bold text-balance">
            Prêt à Découvrir{" "}
            <span className="bg-gradient-to-r from-primary to-chart-1 bg-clip-text text-transparent">
              Notre Univers ?
            </span>
          </h2>

          <p className="text-lg md:text-xl text-muted-foreground max-w-3xl mx-auto text-balance leading-relaxed">
            Plongez dans les détails de notre parcours technique et découvrez comment nous transformons les défis en
            succès
          </p>

          <div className="flex flex-col sm:flex-row items-center justify-center gap-4 pt-8">
            <Link href="/pre-selection">
              <Button
                size="lg"
                className="group text-base px-8 py-6 shadow-xl shadow-primary/30 hover:shadow-2xl hover:shadow-primary/40 transition-all hover:scale-105"
              >
                <Flame className="mr-2 w-5 h-5" />
                Commencer par la pré-sélection
                <ArrowRight className="ml-2 w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Button>
            </Link>
            <Link href="/finale">
              <Button
                size="lg"
                variant="outline"
                className="text-base px-8 py-6 border-2 bg-background/50 backdrop-blur-sm hover:bg-background/80 hover:scale-105 transition-all"
              >
                <Trophy className="mr-2 w-5 h-5" />
                Aller directement à la finale
              </Button>
            </Link>
          </div>

          <div className="pt-12">
            <p className="text-2xl md:text-3xl font-bold bg-gradient-to-r from-primary via-chart-1 to-chart-2 bg-clip-text text-transparent italic">
              "L'innovation commence par la curiosité"
            </p>
          </div>
        </div>
      </div>
    </section>
  )
}
