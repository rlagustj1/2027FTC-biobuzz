plugins {
    alias(libs.plugins.android.application)
}

android {
    namespace = "com.example.a2027ftcbiobuzzseason"
    compileSdk {
        version = release(36) {
            minorApiLevel = 1
        }
    }

    defaultConfig {
        applicationId = "com.example.a2027ftcbiobuzzseason"
        minSdk = 24
        targetSdk = 36
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            optimization {
                enable = false
            }
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
}

dependencies {
    implementation(libs.appcompat)
    implementation(libs.material)
    testImplementation(libs.junit)
    androidTestImplementation(libs.espresso.core)
    androidTestImplementation(libs.ext.junit)

    // FTC SDK
    implementation("org.firstinspires.ftc:RobotCore:10.1.1")
    implementation("org.firstinspires.ftc:Hardware:10.1.1")
    implementation("org.firstinspires.ftc:FtcCommon:10.1.1")
    implementation("org.firstinspires.ftc:Vision:10.1.1")
    implementation("org.firstinspires.ftc:Inspection:10.1.1")

    // FTCLib
    implementation("org.ftclib.ftclib:core:2.1.1")
}