"use client"

import { useEffect, useRef, useState } from "react"
import { Button } from "@/components/ui/button"
import { Card, CardContent } from "@/components/ui/card"
import { ArrowRight, BookOpen, Rocket, Trophy, Calendar } from "lucide-react"
import Link from "next/link"

export function JourneySection() {
  const [visibleSteps, setVisibleSteps] = useState<number[]>([])
  const [hoveredStep, setHoveredStep] = useState<number | null>(null)
  const sectionRef = useRef<HTMLElement>(null)

  const steps = [
    {
      id: 1,
      status: "completed",
      phase: "Phase 1",
      title: "Pré-sélection",
      period: "Juin 2025",
      icon: BookOpen,
      description: "9 projets techniques pour maîtriser les fondamentaux et développer notre méthodologie de travail",
      details: "3 semaines • 3 domaines • 9 défis relevés",
      link: "/pre-selection",
      buttonText: "Explorer la pré-sélection",
      color: "primary"
    },
    {
      id: 2,
      status: "current",
      phase: "Phase 2",
      title: "Finale",
      period: "Juillet 2025",
      icon: Trophy,
      description: "Un système robotique complet qui intègre vision IA, manipulation précise et interface de contrôle",
      details: "Système de tri • ROS Master • DofBot",
      link: "/finale",
      buttonText: "Découvrir le projet final",
      color: "chart-1"
    },
    {
      id: 3,
      status: "upcoming",
      phase: "Phase 3",
      title: "Présentation",
      period: "10 Juillet 2025",
      icon: Calendar,
      description: "Démonstration finale du système complet devant le jury et présentation de notre documentation",
      details: "Démonstration • Évaluation • Résultats",
      link: "#",
      buttonText: "Bientôt disponible",
      color: "chart-2"
    }
  ]

  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            const stepId = parseInt(entry.target.getAttribute('data-step') || '0')
            setVisibleSteps(prev => [...new Set([...prev, stepId])])
          }
        })
      },
      { threshold: 0.3 }
    )

    const stepElements = document.querySelectorAll('[data-step]')
    stepElements.forEach(el => observer.observe(el))

    return () => observer.disconnect()
  }, [])

  const getCircleStyle = (status: string) => {
    switch (status) {
      case 'completed':
        return 'bg-green-500 border-green-500 text-white'
      case 'current':
        return 'bg-primary border-primary text-white'
      case 'upcoming':
        return 'bg-background border-muted text-muted-foreground'
      default:
        return 'bg-background border-muted text-muted-foreground'
    }
  }

  return (
    <section ref={sectionRef} className="relative py-24 px-4 overflow-hidden">
      <div className="absolute inset-0 bg-gradient-to-b from-secondary/20 via-background to-background" />
      <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[800px] h-[800px] bg-primary/5 rounded-full blur-[150px]" />

      <div className="container relative mx-auto max-w-4xl">
        <div className="text-center space-y-4 mb-16">
          <div className="inline-flex items-center justify-center gap-2 mb-4">
            <Rocket className="w-6 h-6 text-primary" />
            <span className="text-sm font-semibold text-primary uppercase tracking-wider">Notre Parcours</span>
          </div>
          <h2 className="text-4xl md:text-5xl lg:text-6xl font-bold text-balance">
            De l'Apprentissage à{" "}
            <span className="bg-gradient-to-r from-primary to-chart-1 bg-clip-text text-transparent">l'Excellence</span>
          </h2>
        </div>

        {/* Clean Timeline */}
        <div className="relative max-w-7xl mx-auto">
          {/* Central timeline line */}
          <div className="absolute left-1/2 top-0 bottom-0 w-1 -translate-x-1/2 bg-muted/20 rounded-full">
            <div 
              className="w-full bg-gradient-to-b from-green-500 via-primary to-chart-2 rounded-full transition-all duration-1000 ease-out"
              style={{ 
                height: `${(visibleSteps.length / steps.length) * 100}%`
              }}
            />
          </div>

          {/* Timeline steps */}
          <div className="relative py-16 space-y-24">
            {steps.map((step, index) => {
              const Icon = step.icon
              const isVisible = visibleSteps.includes(step.id)
              const isHovered = hoveredStep === step.id
              const isOtherHovered = hoveredStep !== null && hoveredStep !== step.id
              const isLeft = index % 2 === 0
              
              return (
                <div
                  key={step.id}
                  data-step={step.id}
                  className={`relative flex items-center transition-all duration-700 ${
                    isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-4'
                  } ${
                    isOtherHovered ? 'opacity-40' : 'opacity-100'
                  } ${
                    isLeft ? 'flex-row' : 'flex-row-reverse'
                  }`}
                  style={{ transitionDelay: `${index * 300}ms` }}
                  onMouseEnter={() => setHoveredStep(step.id)}
                  onMouseLeave={() => setHoveredStep(null)}
                >
                  {/* Main content card */}
                  <div className={`w-[28rem] transition-all duration-500 ${
                    isHovered ? 'transform -translate-y-1 scale-[1.02]' : ''
                  }`}>
                    <Card className={`relative transition-all duration-500 ${
                      isHovered ? 'shadow-xl border-primary/30' : 'shadow-lg'
                    } ${
                      step.status === 'completed' 
                        ? 'border-green-500/20 bg-green-500/5' 
                        : step.status === 'current'
                        ? 'border-primary/20 bg-primary/5'
                        : 'border-muted/20'
                    }`}>
                      <CardContent className="p-6 space-y-4">
                        <div className="flex items-center justify-between">
                          <div>
                            <span className={`text-sm font-semibold uppercase tracking-wider ${
                              step.status === 'completed' ? 'text-green-500' : 
                              step.status === 'current' ? 'text-primary' : 'text-muted-foreground'
                            }`}>
                              {step.phase}
                            </span>
                            <h3 className="text-xl font-bold">{step.title}</h3>
                          </div>
                          <div className="text-right">
                            <div className="text-sm text-muted-foreground">{step.period}</div>
                            <div className="text-xs text-muted-foreground">{step.details}</div>
                          </div>
                        </div>
                        
                        <p className="text-muted-foreground leading-relaxed">
                          {step.description}
                        </p>

                        {step.link !== '#' ? (
                          <Link href={step.link}>
                            <Button 
                              className="w-full group"
                              variant={step.status === 'upcoming' ? 'outline' : 'default'}
                              disabled={step.status === 'upcoming'}
                            >
                              {step.buttonText}
                              <ArrowRight className="ml-2 w-4 h-4 group-hover:translate-x-1 transition-transform" />
                            </Button>
                          </Link>
                        ) : (
                          <Button className="w-full" variant="outline" disabled>
                            {step.buttonText}
                          </Button>
                        )}
                      </CardContent>
                    </Card>
                  </div>

                  {/* Central circle */}
                  <div className="relative z-10 flex-shrink-0 mx-8">
                    <div className={`w-16 h-16 rounded-full border-4 flex items-center justify-center transition-all duration-500 ${getCircleStyle(step.status)} ${
                      isHovered ? 'scale-110 shadow-xl ring-4 ring-primary/20' : 'scale-100'
                    }`}>
                      <Icon className={`transition-all duration-300 ${isHovered ? 'w-7 h-7' : 'w-6 h-6'}`} />
                    </div>
                    
                    {isHovered && (
                      <div className="absolute inset-0 rounded-full border-2 border-primary/30 animate-ping" />
                    )}
                  </div>

                  {/* Side description */}
                  <div className={`w-96 transition-all duration-700 ${
                    isVisible ? 'opacity-100 translate-x-0' : `opacity-0 ${isLeft ? 'translate-x-4' : '-translate-x-4'}`
                  }`} style={{ transitionDelay: `${index * 300 + 150}ms` }}>
                    <div className={`${isLeft ? 'text-left' : 'text-right'} space-y-2`}>
                      <div className={`text-xs uppercase tracking-wider font-semibold ${
                        step.status === 'completed' ? 'text-green-600' : 
                        step.status === 'current' ? 'text-primary' : 'text-muted-foreground'
                      }`}>
                        {step.status === 'completed' ? 'Phase Terminée' : 
                         step.status === 'current' ? 'Phase Actuelle' : 'Phase À Venir'}
                      </div>
                      <p className="text-sm text-muted-foreground leading-relaxed">
                        {step.id === 1 
                          ? "Phase de sélection visant à identifier et trier les candidats les plus méritants. Tests techniques dans trois domaines."
                          : step.id === 2
                          ? "Phase finale où nous travaillons sur le projet complet avec différents outils : ROS Master, DofBot, systèmes de vision."
                          : "Présentation finale du système devant le jury. Démonstration complète du projet et évaluation des résultats."
                        }
                      </p>
                    </div>
                  </div>
                </div>
              )
            })}
          </div>
        </div>
      </div>
    </section>
  )
}
