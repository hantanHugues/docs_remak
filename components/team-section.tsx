import { Card, CardContent } from "@/components/ui/card"
import { Users, Cpu, Code, Wrench } from "lucide-react"

export function TeamSection() {

  return (
    <section className="relative py-24 px-4 overflow-hidden">
      <div className="absolute inset-0 bg-gradient-to-b from-background via-secondary/5 to-background" />
      <div className="absolute top-20 right-10 w-72 h-72 bg-primary/10 rounded-full blur-[100px]" />
      <div className="absolute bottom-20 left-10 w-72 h-72 bg-chart-2/10 rounded-full blur-[100px]" />

      <div className="container relative mx-auto max-w-6xl">
        <div className="text-center space-y-4 mb-16">
          <div className="inline-flex items-center justify-center gap-2 mb-4">
            <Users className="w-6 h-6 text-primary" />
            <span className="text-sm font-semibold text-primary uppercase tracking-wider">Nos Compétences</span>
          </div>
          <h2 className="text-4xl md:text-5xl lg:text-6xl font-bold text-balance">
            Trois Domaines d'Excellence,{" "}
            <span className="bg-gradient-to-r from-primary to-chart-1 bg-clip-text text-transparent">
              Une Expertise Multidisciplinaire
            </span>
          </h2>
          <p className="text-lg md:text-xl text-muted-foreground max-w-3xl mx-auto text-balance leading-relaxed">
            Notre équipe IFRI maîtrise les trois piliers techniques de la robotique moderne. Chaque domaine apporte 
            son expertise spécialisée pour créer des solutions innovantes et documentées.
          </p>
        </div>

        <div className="grid md:grid-cols-3 gap-8 mb-16">
          <Card className="group relative overflow-hidden hover:shadow-2xl transition-all duration-500 hover:-translate-y-2 border-2 hover:border-primary/50 bg-gradient-to-br from-card to-card/50 backdrop-blur-sm">
            <div className="absolute inset-0 bg-gradient-to-br from-primary/10 to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-500" />
            <CardContent className="relative p-8 space-y-6">
              <div className="flex items-center justify-between">
                <div className="w-16 h-16 rounded-2xl bg-gradient-to-br from-primary/20 to-primary/5 flex items-center justify-center text-primary group-hover:scale-110 transition-all duration-500 shadow-lg">
                  <Cpu className="w-8 h-8" />
                </div>
                <div className="text-right">
                  <div className="text-2xl font-bold text-primary">5</div>
                  <div className="text-xs text-muted-foreground">membres</div>
                </div>
              </div>
              <div>
                <h3 className="text-2xl font-bold mb-2">Électronique</h3>
                <p className="text-muted-foreground leading-relaxed mb-4">
                  Conception de circuits, systèmes embarqués et communication inter-microcontrôleurs
                </p>
                <div className="flex flex-wrap gap-2">
                  <span className="px-3 py-1 bg-primary/10 text-primary text-xs rounded-full">PCB Design</span>
                  <span className="px-3 py-1 bg-primary/10 text-primary text-xs rounded-full">Arduino</span>
                  <span className="px-3 py-1 bg-primary/10 text-primary text-xs rounded-full">ESP32</span>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card className="group relative overflow-hidden hover:shadow-2xl transition-all duration-500 hover:-translate-y-2 border-2 hover:border-chart-1/50 bg-gradient-to-br from-card to-card/50 backdrop-blur-sm">
            <div className="absolute inset-0 bg-gradient-to-br from-chart-1/10 to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-500" />
            <CardContent className="relative p-8 space-y-6">
              <div className="flex items-center justify-between">
                <div className="w-16 h-16 rounded-2xl bg-gradient-to-br from-chart-1/20 to-chart-1/5 flex items-center justify-center text-chart-1 group-hover:scale-110 transition-all duration-500 shadow-lg">
                  <Code className="w-8 h-8" />
                </div>
                <div className="text-right">
                  <div className="text-2xl font-bold text-chart-1">4</div>
                  <div className="text-xs text-muted-foreground">membres</div>
                </div>
              </div>
              <div>
                <h3 className="text-2xl font-bold mb-2">Informatique</h3>
                <p className="text-muted-foreground leading-relaxed mb-4">
                  Développement web, intelligence artificielle et systèmes distribués
                </p>
                <div className="flex flex-wrap gap-2">
                  <span className="px-3 py-1 bg-chart-1/10 text-chart-1 text-xs rounded-full">React</span>
                  <span className="px-3 py-1 bg-chart-1/10 text-chart-1 text-xs rounded-full">Node.js</span>
                  <span className="px-3 py-1 bg-chart-1/10 text-chart-1 text-xs rounded-full">IA</span>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card className="group relative overflow-hidden hover:shadow-2xl transition-all duration-500 hover:-translate-y-2 border-2 hover:border-chart-2/50 bg-gradient-to-br from-card to-card/50 backdrop-blur-sm">
            <div className="absolute inset-0 bg-gradient-to-br from-chart-2/10 to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-500" />
            <CardContent className="relative p-8 space-y-6">
              <div className="flex items-center justify-between">
                <div className="w-16 h-16 rounded-2xl bg-gradient-to-br from-chart-2/20 to-chart-2/5 flex items-center justify-center text-chart-2 group-hover:scale-110 transition-all duration-500 shadow-lg">
                  <Wrench className="w-8 h-8" />
                </div>
                <div className="text-right">
                  <div className="text-2xl font-bold text-chart-2">3</div>
                  <div className="text-xs text-muted-foreground">membres</div>
                </div>
              </div>
              <div>
                <h3 className="text-2xl font-bold mb-2">Mécanique</h3>
                <p className="text-muted-foreground leading-relaxed mb-4">
                  Conception CAO, fabrication et assemblage de systèmes mécaniques
                </p>
                <div className="flex flex-wrap gap-2">
                  <span className="px-3 py-1 bg-chart-2/10 text-chart-2 text-xs rounded-full">SolidWorks</span>
                  <span className="px-3 py-1 bg-chart-2/10 text-chart-2 text-xs rounded-full">Impression 3D</span>
                  <span className="px-3 py-1 bg-chart-2/10 text-chart-2 text-xs rounded-full">Usinage</span>
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    </section>
  )
}
