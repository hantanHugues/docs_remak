import { Users, Github, Linkedin, Instagram } from "lucide-react"
import Link from "next/link"

export function ExpertiseSection() {
  const teamMembers = [
    {
      id: 1,
      name: "Alex Martin",
      role: "Chef d'équipe & Développeur Full-Stack",
      domain: "Informatique",
      image: "/api/placeholder/150/150",
      bio: "Passionné par l'IA et les systèmes distribués",
      social: {
        github: "https://github.com/alexmartin",
        linkedin: "https://linkedin.com/in/alexmartin",
        instagram: "https://instagram.com/alexmartin"
      }
    },
    {
      id: 2,
      name: "Sarah Chen",
      role: "Ingénieure Électronique",
      domain: "Électronique",
      image: "/api/placeholder/150/150",
      bio: "Spécialiste en systèmes embarqués et IoT",
      social: {
        github: "https://github.com/sarahchen",
        linkedin: "https://linkedin.com/in/sarahchen",
        instagram: "https://instagram.com/sarahchen"
      }
    },
    {
      id: 3,
      name: "Lucas Dubois",
      role: "Concepteur Mécanique",
      domain: "Mécanique",
      image: "/api/placeholder/150/150",
      bio: "Expert en CAO et fabrication additive",
      social: {
        github: "https://github.com/lucasdubois",
        linkedin: "https://linkedin.com/in/lucasdubois",
        instagram: "https://instagram.com/lucasdubois"
      }
    },
    {
      id: 4,
      name: "Emma Rodriguez",
      role: "Développeuse IA & Vision",
      domain: "Informatique",
      image: "/api/placeholder/150/150",
      bio: "Spécialisée en computer vision et ML",
      social: {
        github: "https://github.com/emmarodriguez",
        linkedin: "https://linkedin.com/in/emmarodriguez",
        instagram: "https://instagram.com/emmarodriguez"
      }
    },
    {
      id: 5,
      name: "Thomas Leroy",
      role: "Ingénieur Systèmes Embarqués",
      domain: "Électronique",
      image: "/api/placeholder/150/150",
      bio: "Expert en microcontrôleurs et communication",
      social: {
        github: "https://github.com/thomasleroy",
        linkedin: "https://linkedin.com/in/thomasleroy",
        instagram: "https://instagram.com/thomasleroy"
      }
    },
    {
      id: 6,
      name: "Léa Moreau",
      role: "Designer UX/UI",
      domain: "Informatique",
      image: "/api/placeholder/150/150",
      bio: "Créatrice d'interfaces intuitives",
      social: {
        github: "https://github.com/leamoreau",
        linkedin: "https://linkedin.com/in/leamoreau",
        instagram: "https://instagram.com/leamoreau"
      }
    },
    {
      id: 7,
      name: "Antoine Blanc",
      role: "Ingénieur Mécatronique",
      domain: "Mécanique",
      image: "/api/placeholder/150/150",
      bio: "Intégration mécanique et électronique",
      social: {
        github: "https://github.com/antoineblanc",
        linkedin: "https://linkedin.com/in/antoineblanc",
        instagram: "https://instagram.com/antoineblanc"
      }
    },
    {
      id: 8,
      name: "Camille Petit",
      role: "Développeuse Backend",
      domain: "Informatique",
      image: "/api/placeholder/150/150",
      bio: "Architecte de systèmes robustes",
      social: {
        github: "https://github.com/camillepetit",
        linkedin: "https://linkedin.com/in/camillepetit",
        instagram: "https://instagram.com/camillepetit"
      }
    },
    {
      id: 9,
      name: "Maxime Roux",
      role: "Ingénieur PCB & Prototypage",
      domain: "Électronique",
      image: "/api/placeholder/150/150",
      bio: "Conception de circuits et prototypage rapide",
      social: {
        github: "https://github.com/maximeroux",
        linkedin: "https://linkedin.com/in/maximeroux",
        instagram: "https://instagram.com/maximeroux"
      }
    },
    {
      id: 10,
      name: "Julie Fabre",
      role: "Ingénieure Fabrication",
      domain: "Mécanique",
      image: "/api/placeholder/150/150",
      bio: "Spécialiste usinage et assemblage",
      social: {
        github: "https://github.com/juliefabre",
        linkedin: "https://linkedin.com/in/juliefabre",
        instagram: "https://instagram.com/juliefabre"
      }
    }
  ]

  const getDomainColor = (domain: string) => {
    switch (domain) {
      case 'Informatique':
        return 'text-chart-2 bg-chart-2/10 border-chart-2/20'
      case 'Électronique':
        return 'text-primary bg-primary/10 border-primary/20'
      case 'Mécanique':
        return 'text-chart-1 bg-chart-1/10 border-chart-1/20'
      default:
        return 'text-muted-foreground bg-muted/10 border-muted/20'
    }
  }

  return (
    <section className="relative py-20 px-4 overflow-hidden">
      {/* Enhanced Background */}
      <div className="absolute inset-0 bg-gradient-to-br from-background via-secondary/10 to-background" />
      <div className="absolute inset-0 bg-[radial-gradient(ellipse_at_top,_var(--tw-gradient-stops))] from-primary/8 via-transparent to-transparent" />
      <div className="absolute top-20 left-10 w-96 h-96 bg-primary/5 rounded-full blur-3xl animate-pulse" />
      <div className="absolute bottom-20 right-10 w-80 h-80 bg-chart-1/5 rounded-full blur-3xl animate-pulse [animation-delay:2s]" />
      <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[600px] h-[600px] bg-chart-2/3 rounded-full blur-[100px] opacity-20" />

      <div className="container relative mx-auto max-w-7xl">
        {/* Enhanced Header */}
        <div className="text-center space-y-6 mb-16">
          <div className="inline-flex items-center justify-center gap-3 mb-6 px-4 py-2 rounded-full bg-primary/10 border border-primary/20">
            <Users className="w-6 h-6 text-primary" />
            <span className="text-sm font-semibold text-primary uppercase tracking-wider">Notre Équipe</span>
          </div>
          <h2 className="text-5xl md:text-6xl lg:text-7xl font-bold text-balance leading-tight">
            Rencontrez les Talents{" "}
            <span className="bg-gradient-to-r from-primary via-chart-1 to-chart-2 bg-clip-text text-transparent">
              Derrière l'Innovation
            </span>
          </h2>
          <p className="text-xl md:text-2xl text-muted-foreground max-w-4xl mx-auto text-balance leading-relaxed">
            Une équipe passionnée de <span className="font-semibold text-primary">10 ingénieurs</span> et développeurs 
            qui allient expertise technique et créativité pour repousser les limites de la robotique.
          </p>
          
          {/* Team Stats */}
          <div className="flex items-center justify-center gap-8 mt-8 pt-8 border-t border-muted/20">
            <div className="text-center">
              <div className="text-3xl font-bold text-primary">10</div>
              <div className="text-sm text-muted-foreground">Membres</div>
            </div>
            <div className="text-center">
              <div className="text-3xl font-bold text-chart-1">3</div>
              <div className="text-sm text-muted-foreground">Domaines</div>
            </div>
            <div className="text-center">
              <div className="text-3xl font-bold text-chart-2">100%</div>
              <div className="text-sm text-muted-foreground">Passion</div>
            </div>
          </div>
        </div>

        {/* Hierarchical Team Layout */}
        <div className="space-y-12">
          {/* Team Leader */}
          {teamMembers.slice(0, 1).map((member, index) => (
            <div key={member.id} className="flex justify-center">
              <div className="group text-center space-y-4">
                {/* Profile Image */}
                <div className="relative mx-auto w-28 h-28">
                  <img 
                    src={member.image} 
                    alt={member.name}
                    className="w-full h-full rounded-full object-cover shadow-lg group-hover:shadow-xl transition-all duration-300 group-hover:scale-105"
                  />
                  <div className={`absolute -inset-1 rounded-full border-3 transition-all duration-300 ${
                    member.domain === 'Informatique' ? 'border-chart-2/50 group-hover:border-chart-2' :
                    member.domain === 'Électronique' ? 'border-primary/50 group-hover:border-primary' : 
                    'border-chart-1/50 group-hover:border-chart-1'
                  }`} />
                </div>

                {/* Name & Role */}
                <div>
                  <h3 className="font-bold text-lg group-hover:text-primary transition-colors duration-300">{member.name}</h3>
                  <p className="text-sm text-muted-foreground mt-1 font-medium">{member.role}</p>
                </div>

                {/* Domain Badge */}
                <div className={`inline-flex items-center px-3 py-1 rounded-full text-xs font-semibold border ${getDomainColor(member.domain)}`}>
                  {member.domain}
                </div>

                {/* Social Links */}
                <div className="flex items-center justify-center gap-3">
                  <Link 
                    href={member.social.github} 
                    className="p-2 rounded-full bg-muted/20 hover:bg-primary/20 hover:text-primary transition-all duration-300 group/social"
                    target="_blank"
                    rel="noopener noreferrer"
                  >
                    <Github className="w-4 h-4 group-hover/social:scale-110 transition-transform" />
                  </Link>
                  <Link 
                    href={member.social.linkedin} 
                    className="p-2 rounded-full bg-muted/20 hover:bg-blue-500/20 hover:text-blue-600 transition-all duration-300 group/social"
                    target="_blank"
                    rel="noopener noreferrer"
                  >
                    <Linkedin className="w-4 h-4 group-hover/social:scale-110 transition-transform" />
                  </Link>
                  <Link 
                    href={member.social.instagram} 
                    className="p-2 rounded-full bg-muted/20 hover:bg-pink-500/20 hover:text-pink-600 transition-all duration-300 group/social"
                    target="_blank"
                    rel="noopener noreferrer"
                  >
                    <Instagram className="w-4 h-4 group-hover/social:scale-110 transition-transform" />
                  </Link>
                </div>
              </div>
            </div>
          ))}

          {/* Team Members - Two Rows Layout */}
          <div className="space-y-16">
            {/* First Row - 4 members */}
            <div className="relative h-56">
              {teamMembers.slice(1, 5).map((member, index) => {
                const positions = ['15%', '35%', '65%', '85%']
                const leftPosition = positions[index]
                
                return (
                  <div 
                    key={member.id} 
                    className="absolute group text-center space-y-3"
                    style={{ 
                      left: leftPosition,
                      transform: 'translateX(-50%)',
                      animationDelay: `${(index + 1) * 150}ms`
                    }}
                  >
                    {/* Profile Image */}
                    <div className="relative mx-auto w-28 h-28">
                      <img 
                        src={member.image} 
                        alt={member.name}
                        className="w-full h-full rounded-full object-cover shadow-lg group-hover:shadow-xl transition-all duration-300 group-hover:scale-105"
                      />
                      <div className={`absolute -inset-1 rounded-full border-3 transition-all duration-300 ${
                        member.domain === 'Informatique' ? 'border-chart-2/50 group-hover:border-chart-2' :
                        member.domain === 'Électronique' ? 'border-primary/50 group-hover:border-primary' : 
                        'border-chart-1/50 group-hover:border-chart-1'
                      }`} />
                    </div>

                    {/* Name & Role */}
                    <div>
                      <h3 className="font-bold text-lg group-hover:text-primary transition-colors duration-300">{member.name}</h3>
                      <p className="text-sm text-muted-foreground mt-1 font-medium">{member.role}</p>
                    </div>

                    {/* Domain Badge */}
                    <div className={`inline-flex items-center px-3 py-1 rounded-full text-xs font-semibold border ${getDomainColor(member.domain)}`}>
                      {member.domain}
                    </div>

                    {/* Social Links */}
                    <div className="flex items-center justify-center gap-3">
                      <Link 
                        href={member.social.github} 
                        className="p-2 rounded-full bg-muted/20 hover:bg-primary/20 hover:text-primary transition-all duration-300 group/social"
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        <Github className="w-4 h-4 group-hover/social:scale-110 transition-transform" />
                      </Link>
                      <Link 
                        href={member.social.linkedin} 
                        className="p-2 rounded-full bg-muted/20 hover:bg-blue-500/20 hover:text-blue-600 transition-all duration-300 group/social"
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        <Linkedin className="w-4 h-4 group-hover/social:scale-110 transition-transform" />
                      </Link>
                      <Link 
                        href={member.social.instagram} 
                        className="p-2 rounded-full bg-muted/20 hover:bg-pink-500/20 hover:text-pink-600 transition-all duration-300 group/social"
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        <Instagram className="w-4 h-4 group-hover/social:scale-110 transition-transform" />
                      </Link>
                    </div>
                  </div>
                )
              })}
            </div>

            {/* Second Row - 5 members */}
            <div className="relative h-56">
              {teamMembers.slice(5).map((member, index) => {
                const positions = ['10%', '30%', '50%', '70%', '90%']
                const leftPosition = positions[index]
                
                return (
                  <div 
                    key={member.id} 
                    className="absolute group text-center space-y-3"
                    style={{ 
                      left: leftPosition,
                      transform: 'translateX(-50%)',
                      animationDelay: `${(index + 5) * 150}ms`
                    }}
                  >
                    {/* Profile Image */}
                    <div className="relative mx-auto w-28 h-28">
                      <img 
                        src={member.image} 
                        alt={member.name}
                        className="w-full h-full rounded-full object-cover shadow-lg group-hover:shadow-xl transition-all duration-300 group-hover:scale-105"
                      />
                      <div className={`absolute -inset-1 rounded-full border-3 transition-all duration-300 ${
                        member.domain === 'Informatique' ? 'border-chart-2/50 group-hover:border-chart-2' :
                        member.domain === 'Électronique' ? 'border-primary/50 group-hover:border-primary' : 
                        'border-chart-1/50 group-hover:border-chart-1'
                      }`} />
                    </div>

                    {/* Name & Role */}
                    <div>
                      <h3 className="font-bold text-lg group-hover:text-primary transition-colors duration-300">{member.name}</h3>
                      <p className="text-sm text-muted-foreground mt-1 font-medium">{member.role}</p>
                    </div>

                    {/* Domain Badge */}
                    <div className={`inline-flex items-center px-3 py-1 rounded-full text-xs font-semibold border ${getDomainColor(member.domain)}`}>
                      {member.domain}
                    </div>

                    {/* Social Links */}
                    <div className="flex items-center justify-center gap-3">
                      <Link 
                        href={member.social.github} 
                        className="p-2 rounded-full bg-muted/20 hover:bg-primary/20 hover:text-primary transition-all duration-300 group/social"
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        <Github className="w-4 h-4 group-hover/social:scale-110 transition-transform" />
                      </Link>
                      <Link 
                        href={member.social.linkedin} 
                        className="p-2 rounded-full bg-muted/20 hover:bg-blue-500/20 hover:text-blue-600 transition-all duration-300 group/social"
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        <Linkedin className="w-4 h-4 group-hover/social:scale-110 transition-transform" />
                      </Link>
                      <Link 
                        href={member.social.instagram} 
                        className="p-2 rounded-full bg-muted/20 hover:bg-pink-500/20 hover:text-pink-600 transition-all duration-300 group/social"
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        <Instagram className="w-4 h-4 group-hover/social:scale-110 transition-transform" />
                      </Link>
                    </div>
                  </div>
                )
              })}
            </div>
          </div>
        </div>
      </div>
    </section>
  )
}
