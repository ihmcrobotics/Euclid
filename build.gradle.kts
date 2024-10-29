plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
}

ihmc {
   group = "us.ihmc"
   version = "0.22.0"
   vcsUrl = "https://github.com/ihmcrobotics/euclid"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.ejml:ejml-core:0.39")
}

geometryDependencies {
   api(ihmc.sourceSetProject("main"))
}

frameDependencies {
   api(ihmc.sourceSetProject("main"))
   api(ihmc.sourceSetProject("geometry"))
}

shapeDependencies {
   api(ihmc.sourceSetProject("main"))
   api(ihmc.sourceSetProject("geometry"))
   api("org.ejml:ejml-ddense:0.39")
}

frameShapeDependencies {
   api(ihmc.sourceSetProject("main"))
   api(ihmc.sourceSetProject("geometry"))
   api(ihmc.sourceSetProject("shape"))
   api(ihmc.sourceSetProject("frame"))
}

testDependencies {
   api(ihmc.sourceSetProject("geometry"))
   api(ihmc.sourceSetProject("frame"))
   api(ihmc.sourceSetProject("shape"))
   api(ihmc.sourceSetProject("frame-shape"))

   api("org.ejml:ejml-ddense:0.39")
   api("us.ihmc:ihmc-commons-testing:0.34.0")
}
