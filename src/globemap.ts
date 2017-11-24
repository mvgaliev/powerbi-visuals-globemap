/*
 *  Power BI Visualizations
 *
 *  Copyright (c) Microsoft Corporation
 *  All rights reserved.
 *  MIT License
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the ""Software""), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED *AS IS*, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */
class GlobeMapHeatMapClass {
    constructor(propertyes: {}) {}
    public display() {}
    public blur() {}
    public update() {}
    public clear() {}
    public addPoint(x: number, y: number, heatPointSize: number, heatIntensity: number) {}
    canvas: HTMLVideoElement;
}
let WebGLHeatmap = <typeof GlobeMapHeatMapClass> window["createWebGLHeatmap"];

module powerbi.extensibility.visual {
    // powerbi.extensibility.geocoder
    import ILocation = powerbi.extensibility.geocoder.ILocation;


    // powerbi.extensibility.utils.formatting
    import LocalStorageService = powerbi.extensibility.utils.formatting.LocalStorageService;
    import IStorageService = powerbi.extensibility.utils.formatting.IStorageService;

    export class MercartorSphere extends THREE.Geometry {
        radius: number;
        widthSegments: number;
        heightSegments: number;
        t: number;
        vertices: THREE.Vector3[];
        prototype: {};
        constructor (radius: number, widthSegments: number, heightSegments: number) {
            debugger;
            super();
            this.radius = radius;
            this.widthSegments = widthSegments;
            this.heightSegments = heightSegments;

            this.t = 0;

            let x: number;
            let y: number;
            const vertices = [];
            const uvs = [];

            function interplolate(a, b, t) {
                return (1 - t) * a + t * b;
            }

            // interpolates between sphere and plane
            function interpolateVertex(u: number, v: number, t: number) {
                const maxLng: number = Math.PI * 2;
                const maxLat: number = Math.PI;

                const sphereX: number = - this.radius * Math.cos(u * maxLng) * Math.sin(v * maxLat);
                const sphereY: number = - this.radius * Math.cos(v * maxLat);
                const sphereZ: number = this.radius * Math.sin(u * maxLng) * Math.sin(v * maxLat);

                const planeX: number = u * this.radius * 2 - this.radius;
                const planeY: number = v * this.radius * 2 - this.radius;
                const planeZ: number = 0;

                const x1: number = interplolate(sphereX, planeX, t);
                const y1: number = interplolate(sphereY, planeY, t);
                const z: number = interplolate(sphereZ, planeZ, t);

                return new THREE.Vector3(x1, y1, z);
            }

            // http://mathworld.wolfram.com/MercatorProjection.html
            // Mercator projection goes form +85.05 to -85.05 degrees
            function interpolateUV(u: number, v: number, t: number) {
                const lat: number = (v - 0.5) * 89.99 * 2 / 180 * Math.PI; // turn from 0-1 into lat in radians
                const sin: number = Math.sin(lat);
                const normalizedV: number = 0.5 + 0.25 * Math.log((1 + sin) / (1 - sin)) / Math.PI;
                return new THREE.Vector2(u, normalizedV); // interplolate(normalizedV1, v, t))
            }

            for (y = 0; y <= heightSegments; y++) {

                const verticesRow: number[] = [];
                const uvsRow: number[] = [];

                for (x = 0; x <= widthSegments; x++) {

                    const u: number = x / widthSegments;
                    const v: number = y / heightSegments;

                    this.vertices.push(interpolateVertex.call(this, u, v, this.t));
                    uvsRow.push(interpolateUV.call(this, u, v, this.t));
                    verticesRow.push(this.vertices.length - 1);
                }

                vertices.push(verticesRow);
                uvs.push(uvsRow);

            }

            for (y = 0; y < this.heightSegments; y++) {

                for (x = 0; x < this.widthSegments; x++) {

                    const v1: number = vertices[y][x + 1];
                    const v2: number = vertices[y][x];
                    const v3: number = vertices[y + 1][x];
                    const v4: number = vertices[y + 1][x + 1];

                    const n1: THREE.Vector3 = this.vertices[v1].clone().normalize();
                    const n2: THREE.Vector3 = this.vertices[v2].clone().normalize();
                    const n3: THREE.Vector3 = this.vertices[v3].clone().normalize();
                    const n4: THREE.Vector3 = this.vertices[v4].clone().normalize();

                    const uv1: THREE.Vector2 = uvs[y][x + 1];
                    const uv2: THREE.Vector2 = uvs[y][x];
                    const uv3: THREE.Vector2 = uvs[y + 1][x];
                    const uv4: THREE.Vector2 = uvs[y + 1][x + 1];

                    this.faces.push(new THREE.Face3(v1, v2, v3, [n1, n2, n3]));
                    this.faces.push(new THREE.Face3(v1, v3, v4, [n1, n3, n4]));

                    this.faceVertexUvs[0].push([uv1.clone(), uv2.clone(), uv3.clone()]);
                    this.faceVertexUvs[0].push([uv1.clone(), uv3.clone(), uv4.clone()]);
                }
            }

            this.computeFaceNormals();
            this.computeVertexNormals();
            this.computeBoundingSphere();
        }
    }

    export class GlobeMap implements IVisual {
        private localStorageService: IStorageService;
        public static MercartorSphere: MercartorSphere;
        private GlobeSettings = {
            autoRotate: false,
            earthRadius: 30,
            cameraRadius: 100,
            earthSegments: 100,
            heatIntensity: 10,
            minHeatIntensity: 2,
            maxHeatIntensity: 10,
            heatPointSize: 7,
            minHeatPointSize: 2,
            maxHeatPointSize: 7,
            heatmapScaleOnZoom: 0.95,
            barWidth: 0.3,
            minBarWidth: 0.01,
            maxBarWidth: 0.3,
            barWidthScaleOnZoom: 0.9,
            barHeight: 5,
            minBarHeight: 0.75,
            maxBarHeight: 5,
            barHeightScaleOnZoom: 0.9,
            rotateSpeed: 0.5,
            zoomSpeed: 0.8,
            cameraAnimDuration: 1000, // ms
            clickInterval: 200 // ms
        };
        private static ChangeDataType: number = 2;
        private static ChangeAllType: number = 62;
        private static DataPointFillProperty: DataViewObjectPropertyIdentifier = {
            objectName: "dataPoint",
            propertyName: "fill"
        };
        private static CountTilesPerSegment: number = 4;
        private layout: VisualLayout;
        private root: JQuery;
        private rendererContainer: JQuery;
        private rendererCanvas: HTMLElement;
        private camera: THREE.PerspectiveCamera;
        private renderer: THREE.WebGLRenderer;
        private scene: THREE.Scene;
        private orbitControls: THREE.OrbitControls;
        private earth: THREE.Mesh | {material};
        private data: GlobeMapData;
        private get settings(): GlobeMapSettings {
            return this.data && this.data.settings;
        }
        private heatmap: GlobeMapHeatMapClass;
        private heatTexture: THREE.Texture;
        private mapTextures: THREE.Texture[];
        public barsGroup: THREE.Object3D;
        private readyToRender: boolean;
        private deferredRenderTimerId: number;
        private globeMapLocationCache: { [i: string]: ILocation };
        private initialLocationsLength: number = 0;
        private renderLoopEnabled = true;
        private needsRender = false;
        private mousePosNormalized: THREE.Vector2;
        private mousePos: THREE.Vector2;
        private rayCaster: THREE.Raycaster;
        private selectedBar: THREE.Object3D;
        private hoveredBar: THREE.Object3D;
        private averageBarVector: THREE.Vector3;
        private controlContainer: HTMLElement;
        public colors: IColorPalette;
        private animationFrameId: number;
        private cameraAnimationFrameId: number;
        public visualHost: IVisualHost;
        private static Unknown: string = "unknown";  

        private static parseSettings(dataView: DataView): GlobeMapSettings {
            return GlobeMapSettings.parse(dataView) as GlobeMapSettings;
        }

        constructor(options: VisualConstructorOptions) {
            this.currentLanguage = options.host.locale;
            this.localStorageService = new LocalStorageService();
            this.root = $("<div>").appendTo(options.element)
                .attr("drag-resize-disabled", "true")
                .css({
                    "position": "absolute"
                });

            this.visualHost = options.host;

            this.layout = new VisualLayout();
            this.readyToRender = false;

            if (!this.globeMapLocationCache) {
                this.globeMapLocationCache = {};
            }

            this.colors = options.host.colorPalette;

            if (window["THREE"]) {
                this.setup();
            }
        }

        private setup(): void {
            this.initScene();
            this.initMercartorSphere();
            this.initTextures().then(
                () => {
                    this.earth = this.createEarth();
                    this.scene.add(<THREE.Mesh> this.earth);
                    this.readyToRender = true;
                });
            this.initRayCaster();
        }
        private static cameraFov: number = 35;
        private static cameraNear: number = 0.1;
        private static cameraFar: number = 10000;
        private static clearColor: number = 0xbac4d2;
        private static ambientLight: number = 0x000000;
        private static tileSize: number = 256;
        private static initialResolutionLevel: number = 2;
        private static maxResolutionLevel: number = 6;
        private static metadataUrl: string = `https://dev.virtualearth.net/REST/V1/Imagery/Metadata/Road?output=json&uriScheme=https&key=${powerbi.extensibility.geocoder.Settings.BingKey}`;
        private static reserveBindMapsMetadata: BingResourceMetadata = {
            imageUrl: "https://{subdomain}.tiles.virtualearth.net/tiles/r{quadkey}.jpeg?g=0&mkt={culture}",
            imageUrlSubdomains: [
                "t1",
                "t2",
                "t3",
                "t4",
                "t5",
                "t6",
                "t7"
            ],
            imageHeight: 256,
            imageWidth: 256
        };
        private currentLanguage: string = "en-GB";
        private static TILE_STORAGE_KEY = "GLOBEMAP_TILES_STORAGE";
        private static TILE_LANGUAGE_CULTURE = "GLOBEMAP_TILE_LANGUAGE_CULTURE";
        private initScene(): void {
            this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
            this.rendererContainer = $("<div>").appendTo(this.root).addClass("globeMapView");

            this.rendererContainer.append(this.renderer.domElement);
            this.rendererCanvas = this.renderer.domElement;
            this.camera = new THREE.PerspectiveCamera(GlobeMap.cameraFov, this.layout.viewportIn.width / this.layout.viewportIn.height, GlobeMap.cameraNear, GlobeMap.cameraFar);
            this.orbitControls = new THREE.OrbitControls(this.camera, this.rendererCanvas);
            this.orbitControls.enablePan = false;
            this.scene = new THREE.Scene();

            this.renderer.setSize(this.layout.viewportIn.width, this.layout.viewportIn.height);
            this.renderer.setClearColor(GlobeMap.clearColor, 1);
            this.camera.position.z = this.GlobeSettings.cameraRadius;
            this.orbitControls.maxDistance = this.GlobeSettings.cameraRadius;
            this.orbitControls.minDistance = this.GlobeSettings.earthRadius + 1;
            this.orbitControls.rotateSpeed = this.GlobeSettings.rotateSpeed;
            this.orbitControls.zoomSpeed = this.GlobeSettings.zoomSpeed;
            this.orbitControls.autoRotate = this.GlobeSettings.autoRotate;

            const ambientLight: THREE.AmbientLight = new THREE.AmbientLight(GlobeMap.ambientLight);

            this.scene.add(ambientLight);

            const render: FrameRequestCallback = () => {
                try {
                    if (this.renderLoopEnabled) {
                        this.animationFrameId = requestAnimationFrame(render);
                    }
                    if (!this.shouldRender()) {
                        return;
                    }
                    this.orbitControls.update();
                    this.setEarthTexture();
                    if (this.heatmap && this.heatmap.display) {
                        this.heatmap.display(); // Needed for IE/Edge to behave nicely
                    }
                    this.renderer.render(this.scene, this.camera);
                    this.needsRender = false;
                } catch (e) {
                    console.error(e);
                }
            };

            this.animationFrameId = requestAnimationFrame(render);
        }

        private shouldRender(): boolean {
            return this.readyToRender && this.needsRender;
        }

        private createEarth(): THREE.Mesh {
            const geometry: MercartorSphere = new MercartorSphere(
                this.GlobeSettings.earthRadius,
                this.GlobeSettings.earthSegments,
                this.GlobeSettings.earthSegments);
            const material: THREE.MeshPhongMaterial = new THREE.MeshPhongMaterial({
                map: this.mapTextures[0],
                side: THREE.DoubleSide,
                shading: THREE.SmoothShading,
                shininess: 1
            });

            const mesh: THREE.Mesh = new THREE.Mesh(geometry, material);
            mesh.add(new THREE.AmbientLight(0xffffff, 0.95));

            return mesh;
        }

        private initTextures(): JQueryPromise<{}> {
            this.mapTextures = [];
            const tileCulture: string = this.localStorageService.getData(GlobeMap.TILE_LANGUAGE_CULTURE);
            let tileCache: TileMap[] = this.localStorageService.getData(GlobeMap.TILE_STORAGE_KEY);
            if (!tileCache || tileCulture !== this.currentLanguage) {
                // Initialize once, since this is a CPU + Network heavy operation.
                return this.getBingMapsServerMetadata()
                    .then((metadata: BingResourceMetadata) => {
                        tileCache = [];
                        let urlTemplate = metadata.imageUrl.replace("{culture}", this.currentLanguage);
                        for (let level: number = GlobeMap.initialResolutionLevel; level <= GlobeMap.maxResolutionLevel; ++level) {
                            let levelTiles = this.generateQuadsByLevel(level, urlTemplate, metadata.imageUrlSubdomains);
                            this.mapTextures.push(this.createTexture(level, levelTiles));
                            tileCache.push(levelTiles);
                        }
                        this.localStorageService.setData(GlobeMap.TILE_STORAGE_KEY, tileCache);
                        this.localStorageService.setData(GlobeMap.TILE_LANGUAGE_CULTURE, this.currentLanguage);
                        return tileCache;
                    });
            } else {
                for (let level: number = GlobeMap.initialResolutionLevel; level <= GlobeMap.maxResolutionLevel; ++level) {
                    this.mapTextures.push(this.createTexture(level, tileCache[level - GlobeMap.initialResolutionLevel]));
                }
                return jQuery.when(tileCache);
            }
        }

        private getBingMapsServerMetadata(): JQueryPromise<BingResourceMetadata> {
            return $.ajax(GlobeMap.metadataUrl)
                .then((data: BingMetadata) => {
                    if (data.resourceSets.length) {
                        let resourceSet = data.resourceSets[0];
                        if (resourceSet && resourceSet.resources.length) {
                            return resourceSet.resources[0];
                        }
                    }
                    throw "Bing Maps API response was changed. Please update code for new version";
                })
                .fail((error: JQueryPromise<{}>) => {
                    console.error(JSON.stringify(error));
                    return GlobeMap.reserveBindMapsMetadata;
                });
        }

        /**
         * Generate Bing tile object by map level
         * @see https://msdn.microsoft.com/en-us/library/bb259689.aspx
         * @private
         * @param {number} level map lavel
         * @param {string} urlTemplate url template
         * @example https://ecn.{subdomain}.tiles.virtualearth.net/tiles/r{quadkey}.jpeg?g=5691&mkt={culture}&shading=hill
         * @param {string[]} subdomains list of subdomauns
         * @returns {{ [quadKey: string]: string }} Object <quadKey> : <image url>
         * @memberOf GlobeMap
         */
        private generateQuadsByLevel(level: number, urlTemplate: string, subdomains: string[]): TileMap {
            const result: TileMap = {};
            let currentSubDomainNumber: number = 0;
            const generateQuard = (currentLevel: number = 0, quadKey: string = ""): void => {
                if (currentLevel < level) {
                    for (let i = 0; i < GlobeMap.CountTilesPerSegment; i++) {
                        generateQuard(currentLevel + 1, `${quadKey}${i}`);
                    }
                } else if (currentLevel === level) {
                    result[quadKey] = urlTemplate.replace("{subdomain}", subdomains[currentSubDomainNumber]).replace("{quadkey}", quadKey);
                    currentSubDomainNumber++;
                    currentSubDomainNumber = currentSubDomainNumber < subdomains.length ? currentSubDomainNumber : 0;
                }
            };
            generateQuard();
            return result;
        }

        public static colors = [
            "green",
            "yellow",
            "blue",
            "red",
            "white",
            "black",
            "grey",
            "purple",
            "orange"
        ];

        private createTexture(level: number, tiles: TileMap): THREE.Texture {
            const numSegments: number = Math.pow(2, level);
            const canvasSize: number = GlobeMap.tileSize * numSegments;
            const canvas: HTMLCanvasElement = document.createElement("canvas");
            canvas.width = canvasSize;
            canvas.height = canvasSize;
            const texture: THREE.Texture = new THREE.Texture(canvas);
            texture.needsUpdate = true;

            const canvasContext: CanvasRenderingContext2D = canvas.getContext("2d");

            let colorsCount = GlobeMap.colors.length;
            let i = 0;

            for (let quadKey in tiles) {
                if (tiles.hasOwnProperty(quadKey)) {
                    debugger
                    const coords: ICanvasCoordinate = this.getCoordByQuadKey(quadKey);
                    const tile: HTMLImageElement = new Image();

                    canvasContext.fillStyle = GlobeMap.colors[i];
                    ++i;
                    if (i === colorsCount) {
                        i = 0;
                    }
                    canvasContext.fillRect(coords.x * GlobeMap.tileSize, coords.y * GlobeMap.tileSize, GlobeMap.tileSize, GlobeMap.tileSize);//.drawImage(tile, coords.x * GlobeMap.tileSize, coords.y * GlobeMap.tileSize, GlobeMap.tileSize, GlobeMap.tileSize);
                }
            }
            /*this.loadTiles(canvas, tiles, () => {
                texture.needsUpdate = true;
                this.needsRender = true;
            });*/
            return texture;
        }

        private getCoordByQuadKey(quard: string): ICanvasCoordinate {
            const last: number = quard.length - 1;
            let x: number = 0;
            let y: number = 0;

            for (let i: number = last; i >= 0; i--) {
                const chr: string = quard.charAt(i);
                const pow: number = Math.pow(2, last - i);
                switch (chr) {
                    case "1": x += pow; break;
                    case "2": y += pow; break;
                    case "3": x += pow; y += pow; break;
                }
            }

            return { x: x, y: y };
        }

        private setEarthTexture(): void {
            // get distance as arbitrary value from 0-1
            if (!this.camera) {
                return;
            }
            
            const maxDistance: number = this.GlobeSettings.cameraRadius - this.GlobeSettings.earthRadius;
            const distance: number = (this.camera.position.length() - this.GlobeSettings.earthRadius) / maxDistance;

            let texture: THREE.Texture = this.mapTextures[0];
            for (let divider: number = GlobeMap.initialResolutionLevel; divider <= GlobeMap.maxResolutionLevel; divider++) {                
                if (distance <= divider / GlobeMap.maxResolutionLevel) {
                    if (GlobeMap.maxResolutionLevel - divider === 3) {
                        debugger;
                    }
                    texture = this.mapTextures[GlobeMap.maxResolutionLevel - divider];
                    break;
                }
            }

            if (this.earth.material.map !== texture) {
                this.earth.material.map = texture;
            }

            if (this.selectedBar) {
                this.orbitControls.rotateSpeed = this.GlobeSettings.rotateSpeed;
            } else {
                this.orbitControls.rotateSpeed = this.GlobeSettings.rotateSpeed * distance;
            }
        }

        public update(options: VisualUpdateOptions): void {
            if (options.dataViews === undefined || options.dataViews === null) {
                return;
            }
            this.layout.viewport = options.viewport;
            this.root.css(this.layout.viewportIn);

            if (this.layout.viewportChanged) {
                if (this.camera && this.renderer) {
                    this.camera.aspect = this.layout.viewportIn.width / this.layout.viewportIn.height;
                    this.camera.updateProjectionMatrix();
                    this.renderer.setSize(this.layout.viewportIn.width, this.layout.viewportIn.height);
                    this.renderer.render(this.scene, this.camera);
                }
            }
        }

        private initRayCaster() {
            this.rayCaster = new THREE.Raycaster();

            const element: HTMLElement = this.root.get(0);
            let mouseDownTime: number;
            const elementStyle: CSSStyleDeclaration = window.getComputedStyle(element);

            $(this.rendererCanvas).on("mousemove", (event: JQueryEventObject) => {
                const elementViewHeight: number = element.offsetHeight - element.offsetTop
                    - parseFloat(elementStyle.paddingTop)
                    - parseFloat(elementStyle.paddingBottom);

                const elementViewWidth: number = element.offsetWidth - element.offsetLeft
                    - parseFloat(elementStyle.paddingLeft)
                    - parseFloat(elementStyle.paddingRight);

                const fractionalPositionX: number = event.offsetX / elementViewWidth;
                const fractionalPositionY: number = event.offsetY / elementViewHeight;

                this.mousePos = new THREE.Vector2(event.clientX, event.clientY);

                // get coordinates in -1 to +1 space
                this.mousePosNormalized = new THREE.Vector2(fractionalPositionX * 2 - 1, -fractionalPositionY * 2 + 1);

                this.needsRender = true;
            }).on("mousedown", (event: JQueryEventObject) => {
                cancelAnimationFrame(this.cameraAnimationFrameId);
                mouseDownTime = Date.now();
            }).on("mouseup", (event: JQueryEventObject) => {
                // Debounce slow clicks
                if ((Date.now() - mouseDownTime) > this.GlobeSettings.clickInterval) {
                    return;
                }

                if (this.hoveredBar && event.shiftKey) {
                    this.selectedBar = this.hoveredBar;
                    this.animateCamera(this.selectedBar.position, () => {
                        if (!this.selectedBar) return;
                        this.orbitControls.target.copy(this.selectedBar.position.clone().normalize().multiplyScalar(this.GlobeSettings.earthRadius));
                        this.orbitControls.minDistance = 1;
                    });
                } else {
                    if (this.selectedBar) {
                        this.animateCamera(this.selectedBar.position, () => {
                            this.orbitControls.target.set(0, 0, 0);
                            this.orbitControls.minDistance = this.GlobeSettings.earthRadius + 1;
                        });
                        this.selectedBar = null;
                    }
                }
            }).on("mousewheel DOMMouseScroll", (e: {originalEvent}) => {
                this.needsRender = true;
            });
        }        

        private animateCamera(to: THREE.Vector3, done?: Function) {

            if (!this.camera) {
                return;
            }
            cancelAnimationFrame(this.cameraAnimationFrameId);
            const startTime: number = Date.now();
            const duration: number = this.GlobeSettings.cameraAnimDuration;
            const endTime: number = startTime + duration;
            const startPos: THREE.Vector3 = this.camera.position.clone().normalize();
            const endPos: THREE.Vector3 = to.clone().normalize();
            const length: number = this.camera.position.length();
            const alpha: number = 2;
            const beta: number = 1.9;
            const easeInOut = (t) => {
                t *= beta;
                if (t < alpha) {
                    return (t * t * t) / beta;
                }
                t -= beta;
                return (t * t * t + beta) / beta;
            };

            const onUpdate: FrameRequestCallback = () => {
                const now: number = Date.now();
                let t: number = (now - startTime) / duration;
                if (t > alpha) {
                    t = alpha;
                }
                t = easeInOut(t);

                const pos: THREE.Vector3 = new THREE.Vector3()
                    .add(startPos.clone().multiplyScalar(alpha - t))
                    .add(endPos.clone().multiplyScalar(t))
                    .normalize()
                    .multiplyScalar(length);

                this.camera.position.set(pos.x, pos.y, pos.z);

                if (now < endTime) {
                    this.cameraAnimationFrameId = requestAnimationFrame(onUpdate);
                } else if (done) {
                    done();
                }

                this.needsRender = true;
            };

            this.cameraAnimationFrameId = requestAnimationFrame(onUpdate);
        }

        public destroy() {
            cancelAnimationFrame(this.animationFrameId);
            cancelAnimationFrame(this.cameraAnimationFrameId);
            clearTimeout(this.deferredRenderTimerId);
            this.renderLoopEnabled = false;
            this.scene = null;
            this.camera = null;
            if (this.renderer) {
                if (this.renderer.context) {
                    const extension: {loseContext} = this.renderer.context.getExtension("WEBGL_lose_context");
                    if (extension) {
                        extension.loseContext();
                    }
                    this.renderer.context = null;
                }
                this.renderer.domElement = null;
            }
            this.renderer = null;
            this.data = null;
            this.barsGroup = null;
            if (this.orbitControls) {
                this.orbitControls.dispose();
            }

            this.orbitControls = null;
            if (this.rendererCanvas) {
                $(this.rendererCanvas)
                    .off("mousemove mouseup mousedown mousewheel DOMMouseScroll");
            }

            this.rendererCanvas = null;

            if (this.root) {
                this.root.empty();
            }
        }
       
        private initMercartorSphere() {
            if (GlobeMap.MercartorSphere) return;

            let  ms = new MercartorSphere(
                this.GlobeSettings.earthRadius,
                this.GlobeSettings.earthSegments,
                this.GlobeSettings.earthSegments);
            ms.prototype = Object.create(THREE.Geometry.prototype);

            GlobeMap.MercartorSphere = ms;
        }        
    }
}
