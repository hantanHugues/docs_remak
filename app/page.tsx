import { Navbar } from "@/components/navbar"
import { HeroSection } from "@/components/hero-section"
import { TeamSection } from "@/components/team-section"
import { EventSection } from "@/components/event-section"
import { JourneySection } from "@/components/journey-section"
import { ExpertiseSection } from "@/components/expertise-section"
import { VisualPreviewSection } from "@/components/visual-preview-section"
import { CtaSection } from "@/components/cta-section"
import { Footer } from "@/components/footer"
import { AnimatedSection } from "@/components/animated-section"

export default function Home() {
  return (
    <main className="min-h-screen">
      <Navbar />
      <HeroSection />
      <AnimatedSection delay={100}>
        <EventSection />
      </AnimatedSection>
      <AnimatedSection delay={150}>
        <TeamSection />
      </AnimatedSection>
      <AnimatedSection delay={100}>
        <JourneySection />
      </AnimatedSection>
      <AnimatedSection delay={150}>
        <ExpertiseSection />
      </AnimatedSection>
      <AnimatedSection delay={100}>
        <VisualPreviewSection />
      </AnimatedSection>
      <AnimatedSection delay={150}>
        <CtaSection />
      </AnimatedSection>
      <Footer />
    </main>
  )
}
