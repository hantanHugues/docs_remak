"use client"

import { useState, useEffect } from "react"
import { Trophy, Target, Users, ExternalLink } from "lucide-react"
import { Button } from "@/components/ui/button"

export function EventSection() {
  const [activeTab, setActiveTab] = useState<"competition" | "objectives" | "participation">("competition")
  const [isAutoPlaying, setIsAutoPlaying] = useState(true)

  const tabs = [
    { id: "competition" as const, label: "Qu'est-ce que le TRC?" },
    { id: "objectives" as const, label: "Objectifs" },
    { id: "participation" as const, label: "Notre Participation" }
  ]

  // Auto-rotation logic
  useEffect(() => {
    if (!isAutoPlaying) return

    const interval = setInterval(() => {
      setActiveTab(current => {
        const currentIndex = tabs.findIndex(tab => tab.id === current)
        const nextIndex = (currentIndex + 1) % tabs.length
        return tabs[nextIndex].id
      })
    }, 4000) // Change every 4 seconds

    return () => clearInterval(interval)
  }, [isAutoPlaying, tabs])

  const handleTabClick = (tabId: "competition" | "objectives" | "participation") => {
    setActiveTab(tabId)
    setIsAutoPlaying(false) // Stop auto-play when user interacts
    
    // Resume auto-play after 10 seconds of inactivity
    setTimeout(() => setIsAutoPlaying(true), 10000)
  }

  const tabContent = {
    competition: {
      title: "TEKBOT Robotics Challenge",
      description: "Une compétition internationale de robotique organisée au Bénin qui rassemble les meilleures équipes d'étudiants pour relever des défis techniques innovants.",
      stats: [
        { label: "Années d'existence", value: "5+" },
        { label: "Équipes participantes", value: "50+" },
        { label: "Universités", value: "15+" }
      ]
    },
    objectives: {
      title: "Mission & Impact",
      description: "Révéler et former les talents en robotique, encourager l'innovation technologique et créer un écosystème dynamique pour l'ingénierie au Bénin et en Afrique.",
      stats: [
        { label: "Projets créés", value: "200+" },
        { label: "Talents formés", value: "500+" },
        { label: "Innovations", value: "100+" }
      ]
    },
    participation: {
      title: "Équipe IFRI - UAC",
      description: "Notre équipe multidisciplinaire de l'Institut de Formation et de Recherche en Informatique représente l'Université d'Abomey-Calavi dans cette compétition prestigieuse.",
      stats: [
        { label: "Membres", value: "12" },
        { label: "Domaines", value: "3" },
        { label: "Statut", value: "Finalistes" }
      ]
    }
  }

  return (
    <section className="relative py-24 px-4 overflow-hidden bg-gradient-to-b from-background to-secondary/20">
      <div className="absolute inset-0 bg-[radial-gradient(ellipse_at_top,_var(--tw-gradient-stops))] from-primary/5 via-transparent to-transparent" />

      <div className="container relative mx-auto max-w-6xl">
        <div className="text-center space-y-4 mb-16">
          <div className="inline-flex items-center justify-center gap-2 mb-4">
            <Trophy className="w-6 h-6 text-primary" />
            <span className="text-sm font-semibold text-primary uppercase tracking-wider">La Compétition</span>
          </div>
          <h2 className="text-4xl md:text-5xl lg:text-6xl font-bold text-balance">
            TEKBOT Robotics{" "}
            <span className="bg-gradient-to-r from-primary to-chart-1 bg-clip-text text-transparent">
              Challenge 2025
            </span>
          </h2>
        </div>

        <div className="grid lg:grid-cols-2 gap-12 items-start">
          {/* Left side - TRC Logo/Image */}
          <div className="relative">
            <div className="aspect-square rounded-3xl bg-gradient-to-br from-primary/20 via-chart-1/20 to-chart-2/20 p-8 backdrop-blur-sm border border-white/10 shadow-2xl">
              <div className="w-full h-full rounded-2xl bg-gradient-to-br from-primary/10 to-chart-1/10 flex items-center justify-center">
                <div className="text-center space-y-4">
                  <div className="w-24 h-24 mx-auto rounded-2xl bg-gradient-to-br from-primary to-chart-1 flex items-center justify-center shadow-xl">
                    <Trophy className="w-12 h-12 text-white" />
                  </div>
                  <div>
                    <h3 className="text-2xl font-bold bg-gradient-to-r from-primary to-chart-1 bg-clip-text text-transparent">
                      TRC 2025
                    </h3>
                    <p className="text-muted-foreground">Robotics Challenge</p>
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Right side - Tabs Content */}
          <div className="space-y-8">
            {/* Tabs Navigation */}
            <div className="space-y-3">
              <div className="relative flex flex-wrap gap-1 p-1 bg-background/50 backdrop-blur-sm rounded-2xl border border-white/10">
                {/* Sliding background indicator */}
                <div 
                  className="absolute top-1 bottom-1 bg-gradient-to-r from-primary to-chart-1 rounded-xl shadow-lg transition-all duration-500 ease-out"
                  style={{
                    left: `${tabs.findIndex(tab => tab.id === activeTab) * (100 / tabs.length)}%`,
                    width: `${100 / tabs.length}%`,
                    transform: 'translateX(4px)',
                    right: '4px'
                  }}
                />
                
                {tabs.map((tab, index) => (
                  <button
                    key={tab.id}
                    onClick={() => handleTabClick(tab.id)}
                    className={`relative z-10 px-4 py-2 rounded-xl text-sm font-medium transition-all duration-300 flex-1 flex items-center justify-center gap-2 ${
                      activeTab === tab.id
                        ? "text-white font-semibold"
                        : "text-muted-foreground hover:text-foreground"
                    }`}
                  >
                    {/* Circular progress indicator */}
                    {activeTab === tab.id && isAutoPlaying && (
                      <div className="relative w-4 h-4">
                        <svg className="w-4 h-4 -rotate-90" viewBox="0 0 16 16">
                          {/* Background circle */}
                          <circle
                            cx="8"
                            cy="8"
                            r="6"
                            stroke="currentColor"
                            strokeWidth="1.5"
                            fill="none"
                            className="opacity-20"
                          />
                          {/* Progress circle */}
                          <circle
                            cx="8"
                            cy="8"
                            r="6"
                            stroke="currentColor"
                            strokeWidth="1.5"
                            fill="none"
                            strokeDasharray="37.7"
                            strokeDashoffset="37.7"
                            className="opacity-80 animate-circle-progress"
                            strokeLinecap="round"
                          />
                        </svg>
                      </div>
                    )}
                    
                    <span>{tab.label}</span>
                  </button>
                ))}
              </div>
              
              {/* Auto-play indicator - moved below tabs */}
              <div className="flex items-center justify-center gap-2">
                <div className={`w-1.5 h-1.5 rounded-full transition-all duration-300 ${
                  isAutoPlaying ? "bg-primary animate-pulse" : "bg-muted-foreground/40"
                }`} />
                <span className={`text-xs transition-colors duration-300 ${
                  isAutoPlaying ? "text-primary" : "text-muted-foreground"
                }`}>
                  {isAutoPlaying ? "Rotation automatique" : "Mode manuel"}
                </span>
              </div>
            </div>

            {/* Custom CSS for circular progress animation */}
            <style jsx>{`
              @keyframes circle-progress {
                from { 
                  stroke-dashoffset: 37.7;
                }
                to { 
                  stroke-dashoffset: 0;
                }
              }
              .animate-circle-progress {
                animation: circle-progress 4s linear infinite;
              }
            `}</style>

            {/* Tab Content */}
            <div className="space-y-6">
              <div key={activeTab} className="animate-in fade-in-50 slide-in-from-right-5 duration-500">
                <h3 className="text-2xl font-bold mb-4">{tabContent[activeTab].title}</h3>
                <p className="text-muted-foreground leading-relaxed text-lg">
                  {tabContent[activeTab].description}
                </p>
              </div>

              {/* Stats */}
              <div className="grid grid-cols-3 gap-4">
                {tabContent[activeTab].stats.map((stat, index) => (
                  <div 
                    key={`${activeTab}-${index}`} 
                    className="text-center p-4 rounded-xl bg-background/50 backdrop-blur-sm border border-white/10 animate-in fade-in-50 slide-in-from-bottom-3 duration-700"
                    style={{ animationDelay: `${index * 100}ms` }}
                  >
                    <div className="text-2xl font-bold bg-gradient-to-r from-primary to-chart-1 bg-clip-text text-transparent">
                      {stat.value}
                    </div>
                    <div className="text-xs text-muted-foreground mt-1">{stat.label}</div>
                  </div>
                ))}
              </div>

              {/* CTA Button */}
              {activeTab === "competition" && (
                <Button variant="outline" className="gap-2 bg-background/50 backdrop-blur-sm hover:bg-background/80">
                  <ExternalLink className="w-4 h-4" />
                  Site officiel TRC
                </Button>
              )}
            </div>
          </div>
        </div>
      </div>
    </section>
  )
}
